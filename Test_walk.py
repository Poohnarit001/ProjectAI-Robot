# tof1_scan_and_move.py
# ใช้ ToF1 (ดิบเป็น mm) + Gimbal สแกนเพื่อประเมินระยะ "ขวา/หน้า/ซ้าย"
# จากนั้นตัดสินใจเคลื่อนที่ของ chassis ตามเงื่อนไขที่กำหนด
#
# มุมที่ใช้สแกน:
#   ขวา (Right) : 90°, 60°
#   หน้า (Front): 0°, 30°, -30°
#   ซ้าย (Left) : -90°, -60°
#
# ลูปการทำงาน:
#   1) สแกน ~5 วินาที -> ได้ค่าเฉลี่ยระยะของ Right/Front/Left (หน่วย m)
#   2) เช็คเงื่อนไขและสั่งเคลื่อนที่
#   3) ทำซ้ำ
#
# เงื่อนไขการเคลื่อนที่ (หน่วย m):
#   1) ถ้า Left < 0.3 และ Right < 0.3       -> เดินหน้า 0.3 m
#   2) ถ้า Left < 0.3 และ Right > 0.3        -> เดินหน้า 0.1 m
#   3) ถ้า Right < 0.3 และ Left > 0.3        -> เดินหน้า 0.1 m
#   4) ถ้า Front อยู่ในช่วง 0.2–0.5 และ Right < 0.3 -> เลี้ยวซ้าย
#   5) ถ้า Front อยู่ในช่วง 0.2–0.5 และ Left  < 0.3 -> เลี้ยวขวา
#
# ความปลอดภัย:
#   - ถ้า Front < 0.2 m (ใกล้มาก) -> ไม่เดินหน้า ให้ "เลี้ยวหลบ" ตามด้านที่ว่างกว่า
#   - หลังสั่งเลี้ยว จะสั่ง gimbal recenter ให้หันไปด้านหน้าตามทิศใหม่เสมอ

import time
from statistics import mean
from robomaster import robot

# ===================== CONFIG =====================
CONN_TYPE = "ap"          # "ap" ต่อ AP หุ่นโดยตรง | "sta" ผ่านเราเตอร์
FREQ_HZ = 10               # ความถี่ที่ subscribe ToF (ครั้ง/วินาที)
SAMPLE_PER_POSE = 3        # อ่านค่ากี่ครั้งต่อหนึ่ง "มุม" (ปรับเพิ่มเพื่อลด noise)
DWELL_SEC = 0.10           # หน่วงหลังหมุน gimbal เพื่อรอหยุดนิ่ง (วินาที)
GIMBAL_SPEED = 200         # ความเร็วหมุน gimbal (deg/s) เร็วขึ้นเพื่อให้ทัน 5 วินาที
SCAN_WINDOW_SEC = 5.0      # ระยะเวลาการสแกนต่อรอบ (วินาที)

# ชุดมุม yaw (องศา) ของแต่ละทิศ
ANGLES_RIGHT = [90, 60]
ANGLES_FRONT = [0, 30, -30]
ANGLES_LEFT  = [-90, -60]

# ค่ากำหนดการตัดสินใจ (เมตร)
SIDE_CLOSE_TH = 0.30       # เกณฑ์ "ชิดผนัง" ด้านซ้าย/ขวา
FRONT_NEAR_MIN = 0.20      # เกณฑ์หน้าใกล้ (ต่ำกว่านี้ให้หลบก่อน)
FRONT_NEAR_MAX = 0.50

# ระยะที่จะสั่งเคลื่อนที่ (เมตร)
STEP_FORWARD_NARROW = 0.30 # ใช้ในกรณีซ้ายและขวาแคบทั้งคู่
STEP_FORWARD_EDGE   = 0.05 # ใช้ในกรณีชิดข้างใดข้างหนึ่ง
TURN_DEG = 30              # องศาที่จะเลี้ยวซ้าย/ขวาเมื่อหน้าติด (ปรับได้)

# ความเร็วการเคลื่อนที่ของ chassis
XY_SPEED = 1            # ความเร็วเคลื่อนที่เชิงเส้น (m/s)
Z_SPEED  = 60              # ความเร็วการหมุน (deg/s)

# เก็บค่าดิบล่าสุดของ ToF1 (mm) จาก callback
latest_tof1_mm = None

# ===================== CALLBACK =====================
def tof_cb(sub_info):
    """
    ถูกเรียกอัตโนมัติเมื่อหุ่นส่งข้อมูล ToF (หน่วย mm) มาใหม่
    sub_info: [tof1, tof2, tof3, tof4] -> เราใช้เฉพาะ tof1 (index 0)
    """
    global latest_tof1_mm
    latest_tof1_mm = float(sub_info[0])  # เก็บค่าดิบ (mm)

# ===================== SCAN HELPERS =====================
def measure_at_angle(ep_gimbal, yaw_deg):
    """
    หมุน gimbal ไปยังมุม yaw_deg (องศา) แล้วอ่านค่า ToF1 หลายครั้ง
    คืนค่า: ระยะ "เฉลี่ย" ของมุมนี้ในหน่วย "เมตร (m)"
    """
    # หมุน gimbal (pitch = 0 เสมอ) แล้วรอให้หยุดนิ่ง
    ep_gimbal.moveto(pitch=0, yaw=yaw_deg,
                     pitch_speed=GIMBAL_SPEED, yaw_speed=GIMBAL_SPEED).wait_for_completed()
    time.sleep(DWELL_SEC)

    # อ่านค่าดิบ mm ซ้ำ ๆ แล้วเฉลี่ย -> แปลงเป็น m
    mm_list = []
    for _ in range(SAMPLE_PER_POSE):
        if latest_tof1_mm is not None:
            mm_list.append(latest_tof1_mm)
        time.sleep(1.0 / max(1, FREQ_HZ))

    if not mm_list:
        return None
    return mean(mm_list) / 1000.0  # mm -> m

def measure_direction(ep_gimbal, angles):
    """
    วัดระยะสำหรับ "หนึ่งทิศ" โดยวนวัดทุกมุมใน angles แล้วเฉลี่ยอีกชั้น
    """
    vals_m = []
    for ang in angles:
        d_m = measure_at_angle(ep_gimbal, ang)
        if d_m is not None:
            vals_m.append(d_m)
    if not vals_m:
        return None
    return mean(vals_m)

def scan_window(ep_gimbal, window_sec=SCAN_WINDOW_SEC):
    """
    สแกนภายในกรอบเวลา window_sec (~5 วินาที):
    - ทำการ "ครบชุดทิศ" (Right -> Front -> Left) กี่ครั้งก็ได้เท่าที่เวลาพอ
    - สุดท้ายเฉลี่ยค่าของแต่ละทิศจากทุกครั้งที่วัดได้ในช่วงเวลา
    เหตุผล: ถ้าบางจังหวะอ่านไม่ทัน/แกว่ง จะมีหลายตัวอย่างช่วยให้ค่ากลางนิ่งขึ้น
    """
    t0 = time.time()
    buf_right, buf_front, buf_left = [], [], []

    while time.time() - t0 < window_sec:
        d_right = measure_direction(ep_gimbal, ANGLES_RIGHT)
        d_front = measure_direction(ep_gimbal, ANGLES_FRONT)
        d_left  = measure_direction(ep_gimbal, ANGLES_LEFT)

        if d_right is not None: buf_right.append(d_right)
        if d_front is not None: buf_front.append(d_front)
        if d_left  is not None: buf_left.append(d_left)

        # ถ้าเวลาใกล้ครบแล้ว ก็ออกจากลูป
        if time.time() - t0 >= window_sec:
            break

    # ถ้าไม่มีข้อมูลเลย ให้ลองวัดด่วนรอบเดียว (กันกรณีพลาด)
    if not buf_right and not buf_front and not buf_left:
        d_right = measure_direction(ep_gimbal, ANGLES_RIGHT)
        d_front = measure_direction(ep_gimbal, ANGLES_FRONT)
        d_left  = measure_direction(ep_gimbal, ANGLES_LEFT)
        if d_right is not None: buf_right.append(d_right)
        if d_front is not None: buf_front.append(d_front)
        if d_left  is not None: buf_left.append(d_left)

    # เฉลี่ยค่าแต่ละทิศ (ถ้าไม่มีข้อมูลทิศใดเลยจะเป็น None)
    avg_right = mean(buf_right) if buf_right else None
    avg_front = mean(buf_front) if buf_front else None
    avg_left  = mean(buf_left)  if buf_left  else None

    # หมุน gimbal กลับหน้า เพื่อพร้อมสำหรับเคลื่อน/รอบถัดไป
    ep_gimbal.moveto(pitch=0, yaw=0,
                     pitch_speed=GIMBAL_SPEED, yaw_speed=GIMBAL_SPEED).wait_for_completed()

    return avg_right, avg_front, avg_left

# ===================== DECISION & MOTION =====================
def decide_and_move(ep_chassis, ep_gimbal, d_right, d_front, d_left):
    """
    ตัดสินใจและสั่งเคลื่อนที่ตามเงื่อนไขที่กำหนด
    - ทุกคำสั่ง move/turn จะ .wait_for_completed() เพื่อให้จบก่อนคาบถัดไป
    - หลัง "เลี้ยว" จะ recenter gimbal เพื่อให้หันไปด้านหน้าทิศใหม่เสมอ (หันตามการเลี้ยว)
    """
    # ความปลอดภัย: ถ้าหน้าใกล้มากกว่าเกณฑ์ (front < 0.2 m) -> ห้ามเดินหน้า ให้เลี้ยวหลบ
    if d_front is not None and d_front < FRONT_NEAR_MIN:
        # เลือกเลี้ยวไปด้านที่ "กว้างกว่า" (ระยะมากกว่า)
        turn_dir = "left" if (d_left or 0) > (d_right or 0) else "right"
        print(f"[SAFETY] Front={d_front:.2f} m ใกล้มาก! เลี้ยว{turn_dir} {TURN_DEG}°")
        if turn_dir == "left":
            ep_chassis.move(x=0, y=0, z=+TURN_DEG, z_speed=Z_SPEED).wait_for_completed()
        else:
            ep_chassis.move(x=0, y=0, z=-TURN_DEG, z_speed=Z_SPEED).wait_for_completed()
        ep_gimbal.recenter().wait_for_completed()
        return  # จบการตัดสินใจรอบนี้

    # เงื่อนไข 4 และ 5: ถ้าหน้าอยู่ในช่วง 0.2–0.5 m ให้เลี้ยวตามด้านชิด
    if d_front is not None and (FRONT_NEAR_MIN <= d_front <= FRONT_NEAR_MAX):
        if d_right is not None and d_right < SIDE_CLOSE_TH:
            print(f"[TURN LEFT] Front={d_front:.2f} m & Right={d_right:.2f} m<0.3 -> เลี้ยวซ้าย {TURN_DEG}°")
            ep_chassis.move(x=0, y=0, z=+TURN_DEG, z_speed=Z_SPEED).wait_for_completed()
            ep_gimbal.recenter().wait_for_completed()
            return
        if d_left is not None and d_left < SIDE_CLOSE_TH:
            print(f"[TURN RIGHT] Front={d_front:.2f} m & Left={d_left:.2f} m<0.3 -> เลี้ยวขวา {TURN_DEG}°")
            ep_chassis.move(x=0, y=0, z=-TURN_DEG, z_speed=Z_SPEED).wait_for_completed()
            ep_gimbal.recenter().wait_for_completed()
            return

    # เงื่อนไข 1: ซ้าย<0.3 และ ขวา<0.3 -> เดินหน้า 0.3 m
    if (d_left is not None and d_left < SIDE_CLOSE_TH) and (d_right is not None and d_right < SIDE_CLOSE_TH):
        print(f"[FWD 0.30] Left={d_left:.2f} m & Right={d_right:.2f} m < 0.3")
        ep_chassis.move(x=+STEP_FORWARD_NARROW, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    # เงื่อนไข 2: ซ้าย<0.3 และ ขวา>0.3 -> เดินหน้า 0.1 m
    if (d_left is not None and d_left < SIDE_CLOSE_TH) and (d_right is not None and d_right > SIDE_CLOSE_TH):
        print(f"[FWD 0.10] Left={d_left:.2f} m <0.3 & Right={d_right:.2f} m >0.3")
        ep_chassis.move(x=+STEP_FORWARD_EDGE, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    # เงื่อนไข 3: ขวา<0.3 และ ซ้าย>0.3 -> เดินหน้า 0.1 m
    if (d_right is not None and d_right < SIDE_CLOSE_TH) and (d_left is not None and d_left > SIDE_CLOSE_TH):
        print(f"[FWD 0.10] Right={d_right:.2f} m <0.3 & Left={d_left:.2f} m >0.3")
        ep_chassis.move(x=+STEP_FORWARD_EDGE, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    # กรณีไม่เข้าเงื่อนไขใดเลย -> เดินหน้าเบา ๆ 0.2 m (ค่าเริ่มต้น)
    print(f"[FWD 0.20 DEFAULT] R={d_right} F={d_front} L={d_left}")
    ep_chassis.move(x=0.20, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()

# ===================== MAIN =====================
def main():
    # เชื่อมต่อหุ่นและโมดูลย่อย
    ep = robot.Robot()
    ep.initialize(conn_type=CONN_TYPE)
    ep_chassis = ep.chassis
    ep_gimbal  = ep.gimbal
    ep_sensor  = ep.sensor

    # ตั้ง gimbal หันหน้าไว้ก่อน
    ep_gimbal.moveto(pitch=0, yaw=0,
                     pitch_speed=GIMBAL_SPEED, yaw_speed=GIMBAL_SPEED).wait_for_completed()

    # สมัครข้อมูล ToF
    ep_sensor.sub_distance(freq=FREQ_HZ, callback=tof_cb)

    print("เริ่มลูป: สแกน 5 วิ → ตัดสินใจ → เคลื่อนที่ → ทำซ้ำ (Ctrl+C เพื่อหยุด)")
    try:
        while True:
            # 1) สแกนภายใน 5 วินาที ได้ค่าเฉลี่ยของแต่ละทิศ (เมตร)
            d_right, d_front, d_left = scan_window(ep_gimbal, SCAN_WINDOW_SEC)
            if None in (d_right, d_front, d_left):
                print("[WARN] สแกนไม่ครบทุกทิศ ลองใหม่ในรอบถัดไป")
                continue

            print(f"[SCAN AVG] Right={d_right:.2f} m | Front={d_front:.2f} m | Left={d_left:.2f} m")

            # 2) ตัดสินใจและสั่งเคลื่อนที่ตามเงื่อนไข
            decide_and_move(ep_chassis, ep_gimbal, d_right, d_front, d_left)

            # 3) (ออปชัน) เว้นระยะสั้น ๆ ก่อนเริ่มสแกนรอบใหม่
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("หยุดตามคำสั่งผู้ใช้")
    finally:
        # เลิก subscribe + recenter + ปิดการเชื่อมต่อ
        try:
            ep_sensor.unsub_distance()
        except Exception:
            pass
        try:
            ep_gimbal.recenter().wait_for_completed()
        except Exception:
            pass
        ep.close()

if __name__ == "__main__":
    main()
