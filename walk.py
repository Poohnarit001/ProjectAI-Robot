# -*- coding: utf-8 -*-
# tof1_scan_and_move_full.py
# ใช้ ToF1 (ดิบเป็น mm) + Gimbal สแกนหลายมุมเพื่อประเมินระยะ "ขวา/หน้า/ซ้าย"
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
#   3) ทำซ้ำ (Ctrl+C เพื่อหยุด)
#
# เงื่อนไขที่ผู้ใช้ต้องการเพิ่ม (มาก่อนกฎเดิม):
#   (NEW-1) ถ้า Front < 0.20 m  -> ถอยหลัง 0.10 m
#   (NEW-2) ถ้า Left  < 0.15 m  -> สไตรฟ์ขวา 0.10 m
#   (NEW-3) ถ้า Right < 0.15 m  -> สไตรฟ์ซ้าย 0.10 m
#
# หมายเหตุ:
# - โค้ดนี้ใช้เฉพาะ ToF1 (index 0) ที่อยู่หน้า gimbal แล้ว "หมุน gimbal" เพื่อมองซ้าย/หน้า/ขวา
# - ค่าดิบจาก SDK เป็น "มิลลิเมตร (mm)" จะแปลงเป็น "เมตร (m)" ก่อนคำนวณ/แสดงผลเสมอ

import time
from statistics import mean
from robomaster import robot

# ===================== CONFIG =====================
CONN_TYPE = "ap"           # "ap" ต่อ AP หุ่นโดยตรง | "sta" ผ่านเราเตอร์
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
SIDE_CLOSE_TH   = 0.30     # เกณฑ์ "ชิดผนัง" ด้านซ้าย/ขวา
FRONT_NEAR_MIN  = 0.20     # เกณฑ์หน้าใกล้มาก (ต่ำกว่านี้ให้เลี่ยงทันที)
FRONT_NEAR_MAX  = 0.50     # ช่วง 0.2–0.5 m เริ่มแคบ (พิจารณาเลี้ยว)

# ระยะที่จะสั่งเคลื่อนที่ (เมตร)
STEP_FORWARD_NARROW = 0.20 # ใช้ในกรณีซ้ายและขวาแคบทั้งคู่ (เดินหน้า)
STEP_STRAFE_EDGE   = 0.10  # ใช้ในกรณีชิดข้างใดข้างหนึ่ง -> สไตรฟ์ไปด้านตรงข้าม
TURN_DEG = 90              # องศาที่จะเลี้ยวซ้าย/ขวาเมื่อหน้าติด (กฎเดิม)

# ความเร็วการเคลื่อนที่ของ chassis
XY_SPEED = 1               # ความเร็วเคลื่อนที่เชิงเส้น (m/s) *แนะนำลดเป็น 0.3–0.5 ในพื้นที่แคบ
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
    ขั้นตอน:
      1) หมุน gimbal -> รอหยุดนิ่ง (DWELL_SEC)
      2) อ่านค่าดิบ mm ซ้ำ SAMPLE_PER_POSE ครั้ง
      3) เฉลี่ย แล้วแปลง mm -> m
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
    เหตุผล: เฉลี่ยสองชั้น (ต่อมุม และต่อทิศ) ช่วยลด noise และทำให้ค่าทิศนิ่งขึ้น
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
    - ทำ "ครบชุดทิศ" (Right -> Front -> Left) ซ้ำให้มากที่สุดเท่าที่เวลาเอื้อ
    - เก็บค่าเฉลี่ยของแต่ละทิศจากหลายครั้งในช่วงเวลา เพื่อให้ค่าทิศนิ่งขึ้น
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

    # กันกรณีพลาด: ถ้าไม่มีข้อมูลเลย ให้ลองวัดด่วนรอบเดียว
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

    # -------------------- เงื่อนไขใหม่ที่ผู้ใช้ร้องขอ --------------------
    # (NEW-1) ถ้า ToF ด้านหน้า < 0.20 m -> "ถอยหลัง" 0.10 m
    if d_front is not None and d_front < 0.20:
        print(f"[NEW RULE] Front={d_front:.2f} m < 0.20 -> BACKWARD 0.10 m")
        ep_chassis.move(x=-0.10, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()
        ep_gimbal.recenter().wait_for_completed()
        return

    # (NEW-2) ถ้า ToF ซ้าย < 0.15 m -> "สไตรฟ์ขวา" 0.10 m (ออกห่างผนังซ้าย)
    if d_left is not None and d_left < 0.15:
        print(f"[NEW RULE] Left={d_left:.2f} m < 0.15 -> STRAFE RIGHT 0.10 m")
        ep_chassis.move(x=0, y=+0.10, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    # (NEW-3) ถ้า ToF ขวา < 0.15 m -> "สไตรฟ์ซ้าย" 0.10 m (ออกห่างผนังขวา)
    if d_right is not None and d_right < 0.15:
        print(f"[NEW RULE] Right={d_right:.2f} m < 0.15 -> STRAFE LEFT 0.10 m")
        ep_chassis.move(x=0, y=-0.10, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return
    # -------------------- จบส่วนที่เพิ่มใหม่ --------------------

    # -------------------- ตรรกะเดิม (คงไว้) --------------------
    # ความปลอดภัย: ถ้าหน้าใกล้มากกว่าเกณฑ์ (front < 0.2 m) -> ห้ามเดินหน้า ให้เลี้ยวหลบ
    if d_front is not None and d_front < FRONT_NEAR_MIN:
        turn_dir = "left" if (d_left or 0) > (d_right or 0) else "right"
        print(f"[SAFETY] Front={d_front:.2f} m ใกล้มาก! เลี้ยว{turn_dir} {TURN_DEG}°")
        if turn_dir == "left":
            ep_chassis.move(x=0, y=0, z=+TURN_DEG, z_speed=Z_SPEED).wait_for_completed()
        else:
            ep_chassis.move(x=0, y=0, z=-TURN_DEG, z_speed=Z_SPEED).wait_for_completed()
        ep_gimbal.recenter().wait_for_completed()
        return

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

    # เงื่อนไข 1 เดิม: ซ้าย<0.3 และ ขวา<0.3 -> เดินหน้า
    if (d_left is not None and d_left < SIDE_CLOSE_TH) and (d_right is not None and d_right < SIDE_CLOSE_TH):
        print(f"[FWD {STEP_FORWARD_NARROW:.2f}] Left={d_left:.2f} m & Right={d_right:.2f} m < 0.3")
        ep_chassis.move(x=+STEP_FORWARD_NARROW, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    # เงื่อนไข 2 เดิม (ปรับพฤติกรรมเป็นสไตรฟ์): ซ้าย<0.3 และขวา>0.3 -> สไตรฟ์ขวา 0.10 m
    if (d_left is not None and d_left < SIDE_CLOSE_TH) and (d_right is not None and d_right > SIDE_CLOSE_TH):
        print(f"[STRAFE RIGHT {STEP_STRAFE_EDGE:.2f}] Left={d_left:.2f} m <0.3 & Right={d_right:.2f} m >0.3")
        ep_chassis.move(x=0, y=+STEP_STRAFE_EDGE, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    # เงื่อนไข 3 เดิม (ปรับพฤติกรรมเป็นสไตรฟ์): ขวา<0.3 และซ้าย>0.3 -> สไตรฟ์ซ้าย 0.10 m
    if (d_right is not None and d_right < SIDE_CLOSE_TH) and (d_left is not None and d_left > SIDE_CLOSE_TH):
        print(f"[STRAFE LEFT {STEP_STRAFE_EDGE:.2f}] Right={d_right:.2f} m <0.3 & Left={d_left:.2f} m >0.3")
        ep_chassis.move(x=0, y=-STEP_STRAFE_EDGE, z=0, xy_speed=XY_SPEED).wait_for_completed()
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
            # 1) สแกนภายใน ~5 วินาที ได้ค่าเฉลี่ยของแต่ละทิศ (เมตร)
            d_right, d_front, d_left = scan_window(ep_gimbal, SCAN_WINDOW_SEC)
            if None in (d_right, d_front, d_left):
                print("[WARN] สแกนไม่ครบทุกทิศ ลองใหม่ในรอบถัดไป")
                continue

            print(f"[SCAN AVG] Right={d_right:.2f} m | Front={d_front:.2f} m | Left={d_left:.2f} m")

            # 2) ตัดสินใจและสั่งเคลื่อนที่ตามเงื่อนไข
            decide_and_move(ep_chassis, ep_gimbal, d_right, d_front, d_left)

            # 3) เว้นระยะสั้น ๆ ก่อนเริ่มสแกนรอบใหม่ (กันคำสั่งติดกันเกินไป)
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
