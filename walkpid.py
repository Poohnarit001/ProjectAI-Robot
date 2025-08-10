# -*-coding:utf-8-*-
# RoboMaster EP: Step-by-step navigation using a gimbal-mounted ToF
# Policy: PRIORITY = LEFT (-90°) > RIGHT (+90°) > FORWARD (0°), each must be > CLEAR_THRESH (0.4 m)
# - Scan gimbal yaw at [-90, 0, +90]  (left, center, right)
# - If LEFT is clear:  rotate -90°, go forward (PID) 0.6 m
# - elif RIGHT clear:  rotate +90°, go forward (PID) 0.6 m
# - elif CENTER clear: go forward (PID) 0.6 m
# - else: rotate to search toward the larger distance, then rescan
#
# ToF unit: convert mm -> m (divide by 1000.0)
# PID forward distance:
#   * Uses chassis odometry if available (preferred)
#   * Falls back to a single move command if odometry not available
#
# Controls:
#   * Ctrl+C to stop safely.

import time
import statistics
from typing import List, Tuple

from robomaster import robot

# ------------------- Tunables -------------------
STEP_M = 0.6                  # target distance per step (meters) — now 0.6 as requested
XY_SPEED = 1.0                # m/s for chassis x/y (upper cap for built-in move)
Z_SPEED = 90                  # deg/s for rotation
PITCH_DEG = 0                 # keep ToF level
SCAN_YAWS = [-90, 0, +90]     # yaw angles (deg) to scan: left(-90), center(0), right(+90)
SAMPLES_PER_POSE = 3          # avg N samples per pose
SAMPLE_DELAY = 0.08           # seconds between samples
CLEAR_THRESH = 0.4            # meters: 0..0.4 = blocked; >0.4 = can move
TOF_INDEX = 0                 # which sensor entry reflects your gimbal ToF (0..3)
MAX_RUNTIME_SEC = None        # None for infinite; or put a number to auto-stop

# PID gains for forward 0.6m
Kp = 1.5
Ki = 0.1
Kd = 0.2

# Limits for PID velocity / segment step
PID_MAX_V = 0.8               # m/s cap when streaming small moves
PID_MIN_SEG = 0.03            # m, minimum segment distance per sub-move
PID_MAX_SEG = 0.20            # m, maximum segment distance per sub-move
PID_STOP_TOL = 0.01           # m, finish threshold
PID_TIMEOUT = 5.0             # s, safety timeout for a 0.6 m step
# ------------------------------------------------

_last_distance = None
_have_odom = False
_pose_x = None  # meters (forward axis in robot frame)

def _sub_data_handler(dist_list):
    global _last_distance
    _last_distance = dist_list

def wait_for_distance(timeout: float = 1.0) -> List[float]:
    """Wait until at least one ToF packet is received; return the latest list (raw units from SDK)."""
    global _last_distance
    start = time.time()
    while _last_distance is None and time.time() - start < timeout:
        time.sleep(0.01)
    if _last_distance is None:
        return [float("inf")] * 4
    return list(_last_distance)

def read_tof_value(avg_n: int = 1, inter_delay: float = 0.05) -> float:
    """Read (and optionally average) the ToF value from the selected index, returned in METERS."""
    vals_mm = []
    for _ in range(max(1, avg_n)):
        dist_list = wait_for_distance(timeout=0.5)
        if 0 <= TOF_INDEX < len(dist_list):
            vals_mm.append(dist_list[TOF_INDEX])
        else:
            vals_mm.append(float('inf'))
        time.sleep(inter_delay)

    # Use median for robustness, then convert to meters
    try:
        raw = statistics.median(vals_mm)
    except statistics.StatisticsError:
        raw = vals_mm[-1] if vals_mm else float("inf")

    if raw == float("inf"):
        return float("inf")
    try:
        return float(raw) / 1000.0  # mm -> m
    except Exception:
        return float("inf")

def scan_with_gimbal(ep_gimbal) -> List[Tuple[int, float]]:
    """Sweep gimbal yaw angles and capture ToF at each angle. Returns list of (yaw_deg, distance_m)."""
    results = []
    for yaw in SCAN_YAWS:
        ep_gimbal.moveto(pitch=PITCH_DEG, yaw=yaw, pitch_speed=120, yaw_speed=180).wait_for_completed()
        time.sleep(0.05)  # settle
        d_m = read_tof_value(avg_n=SAMPLES_PER_POSE, inter_delay=SAMPLE_DELAY)  # in meters
        results.append((yaw, d_m))
    # Return to center for forward movement
    ep_gimbal.moveto(pitch=PITCH_DEG, yaw=0, pitch_speed=120, yaw_speed=180).wait_for_completed()
    return results

def choose_actions(results: List[Tuple[int, float]]) -> List[Tuple[str, float]]:
    """
    PRIORITY: LEFT (-90) > RIGHT (+90) > FORWARD (0).
    Return list of actions e.g. [('z', -90), ('x', STEP_M)] or [('x', STEP_M)] or [('z', +/-90)]
    (NOTE: This version still uses the "rotate opposite" requirement you set earlier.
           If you want same-direction rotation, flip the signs below.)
    """
    dist_map = {yaw: d for yaw, d in results}
    d_center = dist_map.get(0, float('inf'))
    d_left   = dist_map.get(-90, float('inf'))  # left = -90°
    d_right  = dist_map.get(+90, float('inf'))  # right = +90°

    # Left first -> rotate opposite (+90°), then forward
    if d_left > CLEAR_THRESH:
        return [('z', +90.0), ('x', STEP_M)]
    # Then right -> rotate opposite (-90°)
    if d_right > CLEAR_THRESH:
        return [('z', -90.0), ('x', STEP_M)]
    # Finally straight
    if d_center > CLEAR_THRESH:
        return [('x', STEP_M)]

    # If no direction is available, rotate toward larger distance to search (still opposite)
    if d_right >= d_left:
        return [('z', -90.0)]
    else:
        return [('z', +90.0)]

# ---------- Optional Odometry Subscription ----------
def _odom_handler(pose):
    """
    Try to parse chassis position feedback.
    Different SDK versions name this callback payload differently.
    Expecting something like dict or list with x in meters.
    """
    global _pose_x
    try:
        # If pose is a dict {'x': <m>, 'y': <m>, 'z': <deg>} or similar:
        if isinstance(pose, dict) and 'x' in pose:
            _pose_x = float(pose['x'])
        elif isinstance(pose, (list, tuple)) and len(pose) >= 1:
            _pose_x = float(pose[0])
    except Exception:
        pass

def setup_odometry(ep_chassis) -> bool:
    """
    Try to subscribe to chassis position. Returns True if success.
    NOTE: API name may vary. Replace with the correct one in your SDK:
      - ep_chassis.sub_position(freq=10, callback=_odom_handler)
      - or ep_chassis.sub_pose(...)
    """
    global _have_odom
    try:
        # >>>>>> REPLACE with the actual API your SDK provides <<<<<<
        # The following lines are placeholders; comment/uncomment as appropriate.
        # ep_chassis.sub_position(freq=10, callback=_odom_handler)
        # or:
        # ep_chassis.sub_pose(freq=10, callback=_odom_handler)
        # For now, default to False; uncomment the real one to enable.
        _have_odom = False
    except Exception:
        _have_odom = False
    return _have_odom

# ---------- PID Forward (0.6 m) ----------
def pid_drive_forward(ep_chassis, target_distance_m: float, v_cap=PID_MAX_V, timeout_s=PID_TIMEOUT):
    """
    Move forward target_distance_m using a simple PID on odometry x (if available).
    If odometry not available, fall back to a single move command.
    """
    if not _have_odom or _pose_x is None:
        # Fallback: rely on built-in closed-loop distance move
        ep_chassis.move(x=target_distance_m, y=0, z=0, xy_speed=XY_SPEED).wait_for_completed()
        return

    x_start = _pose_x
    integ = 0.0
    prev_err = None
    t0 = time.time()

    while True:
        dx = (_pose_x or x_start) - x_start
        err = target_distance_m - dx

        # Stop conditions
        if abs(err) <= PID_STOP_TOL:
            break
        if time.time() - t0 > timeout_s:
            print("PID forward timeout; stopping early.")
            break

        # PID terms
        integ += err
        deriv = 0.0 if prev_err is None else (err - prev_err)
        prev_err = err

        u = Kp * err + Ki * integ + Kd * deriv  # "desired distance next"
        # Convert to a sub-move segment bounded between min/max
        seg = max(PID_MIN_SEG, min(PID_MAX_SEG, abs(u))) * (1 if u > 0 else -1)

        # Issue a short forward segment; we use move(x=seg) to keep RoboMaster internal control
        ep_chassis.move(x=seg, y=0, z=0, xy_speed=min(v_cap, XY_SPEED)).wait_for_completed()

    # small stop to settle
    time.sleep(0.02)

def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_gimbal  = ep_robot.gimbal
    ep_sensor  = ep_robot.sensor

    try:
        # Home gimbal
        ep_gimbal.moveto(pitch=PITCH_DEG, yaw=0, pitch_speed=120, yaw_speed=180).wait_for_completed()

        # Start ToF subscription
        ep_sensor.sub_distance(freq=10, callback=_sub_data_handler)
        wait_for_distance(timeout=1.0)  # prime the first packet

        # Try odometry subscription (optional; wire to your SDK)
        setup_odometry(ep_chassis)

        t0 = time.time()
        step_count = 0

        while True:
            if MAX_RUNTIME_SEC is not None and (time.time() - t0) > MAX_RUNTIME_SEC:
                print("Reached max runtime, stopping.")
                break

            # 1) Scan
            scan_results = scan_with_gimbal(ep_gimbal)
            print("Scan (yaw_deg, dist_m):", scan_results)

            # 2) Decide -> list of actions
            actions = choose_actions(scan_results)
            print("Actions:", actions)

            # 3) Execute actions in order
            for axis, val in actions:
                if axis == 'x':
                    # PID forward to 0.6 m
                    pid_drive_forward(ep_chassis, target_distance_m=val)
                    step_count += 1
                elif axis == 'z':
                    ep_chassis.move(x=0, y=0, z=val, z_speed=Z_SPEED).wait_for_completed()
                else:
                    print("Unknown axis:", axis)

            # Loop back to rescan after each action sequence

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        try:
            ep_sensor.unsub_distance()
        except Exception:
            pass
        try:
            ep_gimbal.moveto(pitch=PITCH_DEG, yaw=0).wait_for_completed()
        except Exception:
            pass
        ep_robot.close()
        print("Robot closed. Steps taken:", step_count)

if __name__ == "__main__":
    main()
