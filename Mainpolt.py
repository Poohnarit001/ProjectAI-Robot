# plot_map_and_path.py
import csv
import math
import os
import re
import matplotlib.pyplot as plt
from collections import defaultdict

MAP_CSV  = "map_data.csv"
PATH_CSV = "path_log.csv"   # ถ้าไม่มีไฟล์ จะวาดแค่แมป

# ============ ปรับได้ ============
CELL_SIZE_M = 0.6   # ขนาด 1 ช่อง (เมตร)
STEP_STARTS_AT_ZERO = True   # ให้สเต็ปเริ่มที่ 0 แล้วนับเฉพาะ event=move
MARKER_OUTSET_CELLS = 0.14   # ระยะดันดาว “ออกนอกกำแพง” (หน่วยเป็นจำนวนช่อง)
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
            # marker ในไฟล์แมป (ใช้วาดด้วย)
            m = {k: (row.get(f"mark_{k}") or "").strip() for k in "NESW"}
            m = {k:(None if (v=="" or v.upper()=="FALSE") else v) for k,v in m.items()}
            marks[(x,y)] = m
    return cells, marks

def _read_path(filename):
    """อ่าน path + marker_candidate จาก path_log.csv (ทนคอลัมน์สะกดต่างกัน)"""
    if not os.path.exists(filename):
        return [], None, []

    moves = []
    candidates = []
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
                candidates.append({
                    "cell": ( _safe_int(r.get(cx1)), _safe_int(r.get(cy1)) ),
                    "side": (r.get(cdir) or "").strip().upper()[:1],
                    "ids":  (r.get(cids) or "").strip(),
                    "note": (r.get(cnote) or "").strip()
                })
                continue

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

# ====== ฟังก์ชันเกี่ยวกับทิศ/กำแพง/ตำแหน่งบนกำแพง ======
def _direction(a, b):
    """คืนทิศ N/E/S/W ตาม delta ระหว่าง cell a->b (ต้องเป็นเพื่อนบ้าน 4-ทิศ)"""
    (x1, y1), (x2, y2) = a, b
    dx, dy = x2 - x1, y2 - y1
    if dx == 1 and dy == 0:  return "E"
    if dx == -1 and dy == 0: return "W"
    if dx == 0 and dy == 1:  return "N"
    if dx == 0 and dy == -1: return "S"
    return None  # ไม่ใช่เพื่อนบ้าน 4-ทิศ

def _opposite(side):
    return {"N":"S","S":"N","E":"W","W":"E"}.get(side)

def _carve_walls_along_path(cells, path):
    """ลบกำแพงที่อยู่ระหว่างเซลล์ที่มีการเดินผ่าน (a->b)"""
    carved = 0
    skipped = 0
    for seg in path:
        a, b = seg["from"], seg["to"]
        if a not in cells or b not in cells:
            skipped += 1
            continue
        side = _direction(a, b)
        if side is None:
            skipped += 1
            continue
        opp = _opposite(side)
        if cells[a].get(side, False):
            cells[a][side] = False
            carved += 1
        if cells[b].get(opp, False):
            cells[b][opp] = False
            carved += 1
    return carved, skipped

def _wall_midpoint(x,y,side):
    """ตำแหน่งกึ่งกลางกำแพง (ด้านใน cell)"""
    if side=="N": return (x+0.5, y+1.0)
    if side=="S": return (x+0.5, y+0.0)
    if side=="E": return (x+1.0, y+0.5)
    if side=="W": return (x+0.0, y+0.5)
    return (x+0.5, y+0.5)

def _side_normal(side):
    """เวกเตอร์ตั้งฉากออกจาก cell"""
    return {
        "N": (0, +1),
        "S": (0, -1),
        "E": (+1, 0),
        "W": (-1, 0),
    }.get(side, (0,0))

# -------------------- plotting --------------------
def plot_map_and_path(map_csv=MAP_CSV, path_csv=PATH_CSV):
    cells, marks_from_map = _read_map(map_csv)
    path, start_cell, candidates = _read_path(path_csv)

    # 1) เจาะกำแพงตามเส้นทางก่อนวาด
    carved, skipped = _carve_walls_along_path(cells, path)

    fig, ax = plt.subplots(figsize=(7.5,7.5))
    ax.set_aspect("equal")

    # 2) วาดกรอบ cell + ผนัง (หลังจากเจาะแล้ว)
    for (x,y), walls in cells.items():
        # กรอบบาง
        ax.plot([x, x+1], [y, y],     linewidth=1, color="lightgray")
        ax.plot([x, x+1], [y+1, y+1], linewidth=1, color="lightgray")
        ax.plot([x, x],   [y, y+1],   linewidth=1, color="lightgray")
        ax.plot([x+1, x+1], [y, y+1], linewidth=1, color="lightgray")

        # ผนังจริง (กำแพงที่ยังเหลืออยู่)
        if walls.get("N"): ax.plot([x, x+1],[y+1, y+1], linewidth=2, color="red")
        if walls.get("E"): ax.plot([x+1, x+1],[y, y+1], linewidth=2, color="red")
        if walls.get("S"): ax.plot([x, x+1],[y, y],     linewidth=2, color="red")
        if walls.get("W"): ax.plot([x, x],[y, y+1],     linewidth=2, color="red")

    # 3) วาด “เส้นยาวๆ” แสดงทางเดิน (polyline ต่อเนื่องผ่าน center)
    if path:
        pts = []
        first_from = path[0]["from"]
        pts.append(_cell_center(*first_from))
        for seg in path:
            pts.append(_cell_center(*seg["to"]))
        xs, ys = zip(*pts)
        ax.plot(xs, ys, linewidth=2.5, alpha=0.95, color="white")  # เปลี่ยนสีได้ด้วย color="..."

    # 4) เขียนตัวเลขสเต็ปในช่องปลายทาง
    step_labels = defaultdict(list)
    for seg in path:
        step_labels[seg["to"]].append(seg["step"])
    for (x,y), labels in step_labels.items():
        cx, cy = _cell_center(x,y)
        line_h = 0.18
        y0 = cy + (len(labels)-1)*line_h/2.0
        for j,t in enumerate(labels):
            ax.text(cx, y0 - j*line_h, str(t),
                    ha="center", va="center",
                    fontsize=10, color="black",
                    bbox=dict(facecolor="white", alpha=0.9, lw=0, pad=0.8))

    # 5) แสดงจุดเริ่มเป็น “0”
    if path:
        ax.text(*_cell_center(*path[0]["from"]), "0",
                color="green", ha="center", va="center",
                fontsize=10, fontweight="bold")

    # 6) วาด MARKERS (รูปดาว + ค่าด้านล่าง + ทิศ)
    #    รวมทั้งจาก map_data (confirmed) และ marker_candidate (candidate)
    #    - confirmed = สีน้ำเงิน
    #    - candidate = สีเทา
    inferred = []

    # 6.1 จากไฟล์ map_data (ถือว่า confirmed)
    for (x,y), mdict in marks_from_map.items():
        for side in "NESW":
            ids = mdict.get(side)
            if ids:
                inferred.append({"cell": (x,y), "side": side, "ids": ids, "confirmed": True})

    # 6.2 จาก marker_candidate ใน path_log (ถ้ามี)
    for c in candidates:
        (x,y) = c["cell"]
        side  = c["side"] if c["side"] in "NESW" else None
        ids   = (c["ids"] or "").strip()
        if x is None or y is None or not side or not ids:
            continue
        inferred.append({"cell": (x,y), "side": side, "ids": ids, "confirmed": False})

    # 6.3 วาดดาว + ข้อความ
    for mk in inferred:
        (x,y), side, ids, confirmed = mk["cell"], mk["side"], mk["ids"], mk["confirmed"]

        # จุดกึ่งกลางกำแพง แล้วดันออกนอกกำแพงเล็กน้อย
        wx, wy = _wall_midpoint(x, y, side)
        nx, ny = _side_normal(side)
        px, py = wx + nx*MARKER_OUTSET_CELLS, wy + ny*MARKER_OUTSET_CELLS

        # ดาว
        ax.plot(px, py, marker="*", markersize=10,
                color=("tab:blue" if confirmed else "gray"),
                alpha=0.95, zorder=5)

        # ข้อความใต้ดาว: "<ids> (<side>)"
        label = f"{ids} ({side})"
        ax.text(px, py - 0.12, label,
                ha="center", va="top",
                fontsize=9, color="black",
                bbox=dict(facecolor="white", alpha=0.9, lw=0, pad=0.6),
                zorder=6)

    # 7) ตั้งค่าแกน/ขอบเขต
    if cells:
        xs = [x for (x,_) in cells.keys()]
        ys = [y for (_,y) in cells.keys()]
        pad = 0.8
        ax.set_xlim(min(xs)-pad, max(xs)+1+pad)
        ax.set_ylim(min(ys)-pad, max(ys)+1+pad)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(False)
    ax.set_title("Map + Path Polyline + Star Markers (value below with direction)")

    plt.tight_layout()
    plt.show()

    # 8) รายงานผลการเจาะกำแพง (debug)
    print(f"Carved walls: {carved} segments removed; Skipped non-adjacent/invalid moves: {skipped}")

if __name__ == "__main__":
    if not os.path.exists(MAP_CSV):
        raise SystemExit(f"ไม่พบไฟล์ {MAP_CSV}")
    plot_map_and_path(MAP_CSV, PATH_CSV)
