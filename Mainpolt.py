# plot_map_and_path.py
import csv
import math
import os
import re
import matplotlib.pyplot as plt
from collections import defaultdict, Counter

MAP_CSV  = "map_data.csv"
PATH_CSV = "path_log.csv"   # ถ้าไม่มีไฟล์ จะวาดแค่แมป

# ============ ปรับได้ ============
CELL_SIZE_M = 0.6   # ขนาด 1 ช่อง (เมตร) ใช้คุมการดัน "far~Xm" ออกนอกกำแพง
STEP_STARTS_AT_ZERO = True   # ให้สเต็ปเริ่มที่ 0 แล้วนับเฉพาะ event=move
# ===============================

# -------------------- helpers --------------------
def _bool(v):
    if isinstance(v, bool): return v
    s = str(v).strip().lower()
    return s in ("true","1","yes","y")

def _read_map(filename):
    """อ่าน cell + walls + markers จาก map_data.csv"""
    cells = {}
    marks = {}   # {(x,y): {"N": "1;4", ...}}
    with open(filename, newline="", encoding="utf-8") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            x = int(row["x"]); y = int(row["y"])
            walls = {
                "N": _bool(row["wall_N"]),
                "E": _bool(row["wall_E"]),
                "S": _bool(row["wall_S"]),
                "W": _bool(row["wall_W"]),
            }
            cells[(x,y)] = walls
            # marker ในไฟล์แมป (ถ้ามี)
            m = {k: (row.get(f"mark_{k}") or "").strip() for k in "NESW"}
            m = {k:(None if (v=="" or v.upper()=="FALSE") else v) for k,v in m.items()}
            marks[(x,y)] = m
    return cells, marks

def _read_path(filename):
    """อ่าน path + marker_candidate จาก path_log.csv (ทนคอลัมน์สะกดต่างกัน)"""
    if not os.path.exists(filename):
        return [], None, []

    moves = []
    candidates = []  # marker_candidate raw
    start_cell = None

    with open(filename, newline="", encoding="utf-8") as f:
        rdr = csv.DictReader(f)
        cols = {c.lower(): c for c in rdr.fieldnames}

        def col(*names):
            for n in names:
                k = n.lower()
                if k in cols: return cols[k]
            return None

        c_event = col("event")
        c_step  = col("step","seq","index")
        cx1 = col("from_x","x1","sx","start_x")
        cy1 = col("from_y","y1","sy","start_y")
        cx2 = col("to_x","x2","dx","dest_x","end_x")
        cy2 = col("to_y","y2","dy","dest_y","end_y")
        cdir = col("dir","direction")
        cids = col("ids")
        cnote = col("note")

        step_counter = -1 if STEP_STARTS_AT_ZERO else 0

        for r in rdr:
            ev = (r.get(c_event) or "").strip().lower() if c_event else ""
            if ev == "marker_candidate":
                # เก็บไว้ตีความทีหลัง
                cand = {
                    "cell": ( _safe_int(r.get(cx1)), _safe_int(r.get(cy1)) ),
                    "side": (r.get(cdir) or "").strip().upper()[:1],
                    "ids":  (r.get(cids) or "").strip(),
                    "note": (r.get(cnote) or "").strip()
                }
                candidates.append(cand)
                continue

            # move เท่านั้นนับเป็นสเต็ป
            if ev != "move":
                continue

            fx, fy = _safe_int(r.get(cx1)), _safe_int(r.get(cy1))
            tx, ty = _safe_int(r.get(cx2)), _safe_int(r.get(cy2))
            if fx is None or fy is None or tx is None or ty is None:
                continue
            if (fx,fy) == (tx,ty):
                continue  # ไม่ถือว่า move

            step_counter += 1
            if start_cell is None:
                start_cell = (fx,fy)

            moves.append({
                "step": step_counter,
                "from": (fx,fy),
                "to":   (tx,ty),
                "dir":  (r.get(cdir) or "")
            })

    return moves, start_cell, candidates

def _safe_int(v):
    if v is None or v == "": return None
    try:
        return int(float(v))
    except Exception:
        return None

def _cell_center(x,y): return x+0.5, y+0.5

def _wall_midpoint(x,y,side, inset=0.0):
    """ตำแหน่งกึ่งกลางกำแพง (ด้านใน cell) + inset เข้าด้านใน"""
    if side=="N": return (x+0.5, y+1-inset)
    if side=="S": return (x+0.5, y+inset)
    if side=="E": return (x+1-inset, y+0.5)
    if side=="W": return (x+inset,   y+0.5)
    return (x+0.5, y+0.5)

def _side_normal(side):
    # เวกเตอร์ตั้งฉากออกนอก cell
    return {
        "N": (0, +1),
        "S": (0, -1),
        "E": (+1, 0),
        "W": (-1, 0),
    }.get(side, (0,0))

_far_re = re.compile(r"far~\s*([0-9]*\.?[0-9]+)\s*m", re.I)

def _parse_far_m(note):
    if not note: return None
    m = _far_re.search(note)
    if not m: return None
    try:
        return float(m.group(1))
    except Exception:
        return None

# -------------------- plotting --------------------
def plot_map_and_path(map_csv=MAP_CSV, path_csv=PATH_CSV):
    cells, marks_from_map = _read_map(map_csv)
    path, start_cell, candidates = _read_path(path_csv)

    fig, ax = plt.subplots(figsize=(7.5,7.5))
    ax.set_aspect("equal")

    # --- ร่างกรอบ cell + ผนัง (แดง) ---
    for (x,y), walls in cells.items():
        # กรอบบาง
        ax.plot([x, x+1], [y, y],   color="lightgray", linewidth=1)
        ax.plot([x, x+1], [y+1, y+1], color="lightgray", linewidth=1)
        ax.plot([x, x],   [y, y+1], color="lightgray", linewidth=1)
        ax.plot([x+1, x+1], [y, y+1], color="lightgray", linewidth=1)

        # ผนังจริง
        if walls.get("N"): ax.plot([x, x+1],[y+1, y+1], color="red", linewidth=2)
        if walls.get("E"): ax.plot([x+1, x+1],[y, y+1], color="red", linewidth=2)
        if walls.get("S"): ax.plot([x, x+1],[y, y], color="red", linewidth=2)
        if walls.get("W"): ax.plot([x, x],[y, y+1], color="red", linewidth=2)

    # --- เส้นทาง: ขยับออกเล็กน้อยถ้าเส้นซ้ำ ---
    pair_counts = Counter()
    step_labels = defaultdict(list)

    def _offset_line(p1, p2, k, gap=0.06):
        (x1,y1),(x2,y2) = p1,p2
        vx, vy = x2-x1, y2-y1
        L = math.hypot(vx, vy)
        if L == 0: return (x1,y1,x2,y2)
        nx, ny = -vy/L, vx/L
        dx, dy = nx*gap*k, ny*gap*k
        return (x1+dx, y1+dy, x2+dx, y2+dy)

    for seg in path:
        i  = seg["step"]
        a  = seg["from"]; b = seg["to"]
        pair = tuple(sorted([a,b]))
        pair_counts[pair] += 1
        k = pair_counts[pair] - 1

        x1,y1 = _cell_center(*a)
        x2,y2 = _cell_center(*b)
        ox1, oy1, ox2, oy2 = _offset_line((x1,y1),(x2,y2), k)

        ax.plot([ox1, ox2], [oy1, oy2], linewidth=2, alpha=0.8)
        step_labels[b].append(str(i))

    # --- เขียนหมายเลขสเต็ปใน “ช่องปลายทาง” แบบหลายบรรทัดตัวเล็ก ---
    for (x,y), labels in step_labels.items():
        cx, cy = _cell_center(x,y)
        line_h = 0.18
        y0 = cy + (len(labels)-1)*line_h/2.0
        for j,t in enumerate(labels):
            ax.text(cx, y0 - j*line_h, t,
                    ha="center", va="center",
                    fontsize=9,
                    bbox=dict(facecolor="white", alpha=0.85, lw=0))

    # --- วาง marker: เอาตัวเลขไป “บนกำแพง” แล้วดันออกนอกกำแพงตาม far~Xm ---
    # รวม marker จาก map (หากมี) + inferred จาก candidates
    inferred = []  # รายการที่จะวาด: dict(cell, side, ids, far_m, confirmed)
    # 1) จากไฟล์ map_data (ถือว่า confirmed, ไม่มีระยะ far)
    for (x,y), mdict in marks_from_map.items():
        for side in "NESW":
            ids = mdict.get(side)
            if ids:
                inferred.append({"cell": (x,y), "side": side, "ids": ids, "far_m": None, "confirmed": True})

    # 2) จาก marker_candidate (far)
    for c in candidates:
        (x,y) = c["cell"]
        side  = c["side"] if c["side"] in "NESW" else None
        ids   = (c["ids"] or "").strip()
        if x is None or y is None or not side or not ids:
            continue
        far_m = _parse_far_m(c.get("note",""))
        inferred.append({"cell": (x,y), "side": side, "ids": ids, "far_m": far_m, "confirmed": False})

    # เขียนจริง
    for mk in inferred:
        (x,y), side, ids, far_m, confirmed = mk["cell"], mk["side"], mk["ids"], mk["far_m"], mk["confirmed"]
        # กึ่งกลางกำแพง (ชิดด้านใน)
        wx, wy = _wall_midpoint(x, y, side, inset=0.0)

        # ระยะดันออกนอกกำแพง (หน่วย "ช่อง")
        nx, ny = _side_normal(side)
        if far_m is None:
            offset_cells = 0.12  # ยื่นออกเล็กน้อยให้พ้นเส้นผนัง
        else:
            # จากเมตร -> ช่อง แล้วจำกัดไม่ให้ข้าม cell ถัดไป
            offset_cells = min(0.48, (far_m / max(1e-6, CELL_SIZE_M)))
            # เพิ่มระยะเล็กน้อยให้พ้นเส้นผนัง
            offset_cells = max(offset_cells, 0.12)

        px, py = wx + nx*offset_cells, wy + ny*offset_cells

        # วาดเส้นนำทางจากกำแพง -> จุด (ให้เห็นว่าดันออก)
        ax.plot([wx, px], [wy, py], linestyle=":", linewidth=1)

        # วาดวงกลมตำแหน่ง + ตัวเลข
        ax.plot(px, py, marker="o",
                markersize=5.5 if confirmed else 5,
                color=("tab:blue" if confirmed else "gray"),
                alpha=0.95 if confirmed else 0.85)

        # ถ้า ids มีหลายค่า ใช้คั่นด้วย ';'
        txt = ids
        ax.text(px, py,
                txt,
                ha="center", va="center",
                fontsize=8,
                color=("tab:blue" if confirmed else "black"),
                bbox=dict(facecolor="white", alpha=0.9, lw=0, pad=0.8))

    # --- START/END label (ถ้ามี path) ---
    if path:
        ax.text(*_cell_center(*path[0]["from"]), "START",
                color="green", ha="center", va="center", fontsize=10, fontweight="bold")
        ax.text(*_cell_center(*path[-1]["to"]), "END",
                color="purple", ha="center", va="center", fontsize=10, fontweight="bold")

    # --- ตั้งค่าแกน/ขอบเขต ---
    if cells:
        xs = [x for (x,_) in cells.keys()]
        ys = [y for (_,y) in cells.keys()]
        pad = 0.8
        ax.set_xlim(min(xs)-pad, max(xs)+1+pad)
        ax.set_ylim(min(ys)-pad, max(ys)+1+pad)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(False)
    ax.set_title("Map + Path + Wall-anchored Markers (pushed outward by far~Xm)")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if not os.path.exists(MAP_CSV):
        raise SystemExit(f"ไม่พบไฟล์ {MAP_CSV}")
    plot_map_and_path(MAP_CSV, PATH_CSV)
