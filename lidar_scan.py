#!/usr/bin/env python3
"""
lidar_scan.py
==================================================================
Everything needed on the Raspberry Pi for the 650 nm line-laser
scanner, in ONE file: laser-line detection, camera + laser-plane
calibration, and the scan itself.

------------------------------------------------------------------
SETUP ON A FRESH RASPBERRY PI IMAGE (Raspberry Pi OS Bookworm, 64-bit)
------------------------------------------------------------------
1. Flash the SD card
   Use Raspberry Pi Imager (https://www.raspberrypi.com/software/).
   Choose "Raspberry Pi OS (64-bit)". The full desktop image is
   easiest since libcamera/picamera2 come preinstalled; the Lite
   image works too but needs one extra apt install (step 4).
   In the Imager's settings (gear icon), set hostname, enable SSH,
   and configure Wi-Fi before writing, so it's headless-ready on
   first boot.

2. First boot
       ssh pi@<hostname-or-ip>
       sudo apt update && sudo apt full-upgrade -y
       sudo reboot

3. Confirm the camera is detected
   Bookworm uses libcamera by default -- nothing to toggle in
   raspi-config.
       rpicam-hello --list-cameras
   (older images may call this libcamera-hello instead)

4. Install dependencies
   Lite image only (full desktop image already has this):
       sudo apt install -y python3-picamera2 --no-install-recommends
   Both image variants:
       sudo apt install -y python3-opencv python3-numpy python3-serial

   Bookworm's system Python is "externally managed" (PEP 668), so a
   plain `pip install` will refuse to run. Installing via apt (above)
   is the simplest fix, and is what picamera2 needs anyway -- it
   isn't reliably pip-installable outside the Pi's own apt repo.

5. Serial port permission
       sudo usermod -aG dialout $USER
   Log out and back in (or reboot) for this to take effect, or you'll
   get a permission-denied error opening the ESP32's port.

6. Wiring recap
       ESP32  <-- USB -->  Raspberry Pi
       Camera Module 2 --> Pi CSI port
       Laser module     --> its own driver/supply, aimed to cross
                             the camera's field of view
   The ESP32 firmware is flashed separately from the Arduino side --
   this script only talks to it over serial once it's running.

7. Find the ESP32's serial port
       ls /dev/ttyUSB* /dev/ttyACM*
   Pass whichever shows up with --port (see USAGE below).

8. Find your laptop's IP (for the point-cloud viewer)
   On the laptop: `ipconfig` (Windows) or `ifconfig` / `ip a`
   (Mac/Linux). Pass it with --laptop-ip. The laptop must already be
   running viewer.py and listening BEFORE you start a scan.

9. Put this file in its own directory on the Pi -- calibrate and
   scan both read/write their .npy files next to it automatically.

------------------------------------------------------------------
USAGE
------------------------------------------------------------------
    python3 lidar_scan.py check                  # verify camera + ESP32 before anything else
    python3 lidar_scan.py calibrate              # run once -> writes the 3 .npy files
    python3 lidar_scan.py scan                    # run per scan (viewer.py must already be listening)

    python3 lidar_scan.py check   --port /dev/ttyUSB0
    python3 lidar_scan.py scan    --port /dev/ttyUSB0 --laptop-ip 192.168.1.100

Files produced by `calibrate`, consumed by `scan` (kept next to this script):
    camera_matrix.npy
    dist_coeffs.npy
    laser_plane.npy
==================================================================
"""

import argparse
import os
import socket
import struct
import time
import traceback

import numpy as np
import cv2

# picamera2 / pyserial are imported lazily inside the functions that need
# them, so `--help` still works on a fresh image before those are set up.


# ============================================================
# Config -- edit these, or override with CLI flags where noted
# ============================================================

# -- Laser detection (650 nm red laser, works directly on the red channel) --
RED_THRESHOLD  = 180   # 0-255; raise if noisy, lower if the line disappears
MIN_ROW_PIXELS = 2     # ignore rows with fewer than this many red pixels
ROW_STRIDE     = 1     # process every Nth row (1 = every row)

# -- Camera intrinsics calibration --
CHESSBOARD     = (9, 6)     # inner corners
SQUARE_SIZE_MM = 25.0
IMAGE_SIZE     = (1280, 720)

# -- Laser plane calibration --
MIN_LASER_DEPTHS = 3   # need >=3 depths for a well-conditioned plane fit

# -- Scan-head mechanical rotation axis, in the CAMERA's own coordinate
#    frame (OpenCV convention: +X right, +Y down, +Z forward/depth), NOT
#    world coordinates. Default assumes a normally-mounted (not rolled)
#    camera looking ACROSS a vertical shaft, so the shaft appears along
#    the camera's own Y axis. If your mount differs:
#      - camera looks straight down the shaft (rare)   -> (0, 0, 1)
#      - camera looks across a horizontal shaft          -> (1, 0, 0)
#      - tilted mount                                     -> measure it
#    Verify by scanning a tall, flat, rigid object: correct axis -> it
#    stays straight and rigid across all angles; wrong axis -> it shears
#    or leans as the angle sweeps.
SCAN_ROTATION_AXIS_CAMERA_FRAME = (0.0, 1.0, 0.0)

# -- ESP32 link (override with --port) --
ESP_PORT = "/dev/ttyUSB0"
ESP_BAUD = 115200
READY_TIMEOUT_S  = 8.0
SETTLE_TIMEOUT_S = 30.0

# -- Viewer link (override with --laptop-ip) --
LAPTOP_IP   = "192.168.1.100"
LAPTOP_PORT = 5005

# -- Scan sweep --
SCAN_MIN_DEG  = -20.0
SCAN_MAX_DEG  =  20.0
SCAN_STEP_DEG =   0.2

# -- Closed-loop motion settings pushed to the ESP32 at scan start --
ESP_SPEED   = 600     # steps/sec motor
ESP_ACCEL   = 300     # steps/sec^2
ESP_TOL_DEG = 0.008   # output-shaft closed-loop tolerance
ESP_MAX_ATT = 20


# ============================================================
# Shared: laser detection, triangulation, rotation
# ============================================================

def detect_laser_line_red(bgr):
    """Extract sub-pixel (u, v) for the laser stripe, one point per row."""
    red = bgr[:, :, 2]                       # OpenCV BGR order
    mask = (red >= RED_THRESHOLD).astype(np.uint8)

    pts = []
    h, w = mask.shape
    for v in range(0, h, ROW_STRIDE):
        row = mask[v]
        idx = np.flatnonzero(row)
        if len(idx) < MIN_ROW_PIXELS:
            continue
        w_row = red[v, idx].astype(np.float32)
        u = float(np.sum(idx * w_row) / np.sum(w_row))
        pts.append((u, v))
    return np.array(pts, dtype=np.float32)


def triangulate(pts_uv, K, D, plane):
    """Convert image points to 3D points in the camera frame."""
    if len(pts_uv) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    pts = pts_uv.reshape(-1, 1, 2).astype(np.float32)
    und = cv2.undistortPoints(pts, K, D)
    x_n = und[:, 0, 0]
    y_n = und[:, 0, 1]

    a, b, c, d = plane
    denom = a * x_n + b * y_n + c
    valid = np.abs(denom) > 1e-6
    t = np.zeros_like(denom)
    t[valid] = -d / denom[valid]
    keep = (t > 0) & valid

    P = np.stack([x_n[keep] * t[keep],
                  y_n[keep] * t[keep],
                  t[keep]], axis=1)
    return P.astype(np.float32)


def rotate_about_axis(P, angle_deg, axis):
    """Rotate (N,3) points about an arbitrary axis (Rodrigues' formula)."""
    ax = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(ax)
    if norm < 1e-12:
        raise ValueError("rotation axis must be non-zero")
    ax = ax / norm

    a = np.deg2rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    Kx = np.array([[0.0, -ax[2], ax[1]],
                   [ax[2], 0.0, -ax[0]],
                   [-ax[1], ax[0], 0.0]])
    R = np.eye(3) + s * Kx + (1.0 - c) * (Kx @ Kx)
    return (P.astype(np.float64) @ R.T).astype(np.float32)


def rotate_about_scan_axis(P, angle_deg):
    """Rotate camera-frame points into the world/scan frame using
    SCAN_ROTATION_AXIS_CAMERA_FRAME (see config above)."""
    return rotate_about_axis(P, angle_deg, SCAN_ROTATION_AXIS_CAMERA_FRAME)


# ============================================================
# Camera helper
# ============================================================

def make_camera(still=False):
    from picamera2 import Picamera2
    cam = Picamera2()
    cfg = (cam.create_still_configuration(main={"size": IMAGE_SIZE, "format": "RGB888"})
           if still else
           cam.create_preview_configuration(main={"size": IMAGE_SIZE, "format": "RGB888"}))
    cam.configure(cfg)
    cam.start()
    time.sleep(1.0)
    return cam


def grab(cam):
    return cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)


# ============================================================
# ESP32 serial helpers
# ============================================================

def open_esp_serial(port):
    import serial
    return serial.Serial(port, ESP_BAUD, timeout=3)


def wait_for_ready(ser, timeout_s=READY_TIMEOUT_S):
    """
    Block until the ESP32's boot banner ("READY,...") arrives.

    Opening the serial port auto-resets the ESP32 (DTR toggle), which
    then reboots and eventually prints READY. Waiting for the actual
    banner (instead of a fixed sleep) avoids a race where a late READY
    line gets consumed as the reply to the first real command, silently
    desyncing every command/response pair for the rest of the run.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("READY"):
            return True
    return False


def esp_send(ser, cmd):
    ser.write((cmd + "\n").encode())
    reply = ser.readline().decode(errors="ignore").strip()
    if not reply.startswith("OK,"):
        print(f"  WARN unexpected reply to '{cmd}': '{reply}'")
    return reply


def esp_wait_settled(ser, timeout_s=SETTLE_TIMEOUT_S):
    """Block until the ESP32 reports the closed-loop move has settled,
    or bail out after timeout_s so one stuck move can't hang the scan."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("OK,SETTLED"):
            return True
        if line.startswith("OK,GAVEUP"):
            print("  WARN closed loop gave up:", line)
            return False
        if line.startswith("LIMIT,STOP"):
            print("  LIMIT TRIP:", line)
            return False
    print(f"  WARN no settle reply within {timeout_s:.0f}s -- skipping this angle")
    return False


# ============================================================
# Mode: check  (quick hardware sanity check)
# ============================================================

def cmd_check(args):
    ok = True

    print("== Camera ==")
    try:
        cam = make_camera(still=False)
        frame = grab(cam)
        pts = detect_laser_line_red(frame)
        print(f"  captured {frame.shape[1]}x{frame.shape[0]} frame, "
              f"{len(pts)} laser points detected at RED_THRESHOLD={RED_THRESHOLD}")
        if len(pts) == 0:
            print("  WARN no laser points -- aim the laser into frame, "
                  "or lower RED_THRESHOLD")
        cam.stop()
    except Exception as e:
        print(f"  FAIL camera: {e}")
        ok = False

    print("== ESP32 ==")
    try:
        ser = open_esp_serial(args.port)
        if wait_for_ready(ser):
            print("  READY banner received")
        else:
            print("  WARN no READY banner -- check wiring/port, continuing anyway")
        print("  " + esp_send(ser, "?"))
        ser.close()
    except Exception as e:
        print(f"  FAIL serial ({args.port}): {e}")
        ok = False

    print("\ncheck " + ("PASSED" if ok else "FAILED"))


# ============================================================
# Mode: calibrate
# ============================================================

def calibrate_camera():
    cam = make_camera(still=False)

    objp = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    objpoints, imgpoints = [], []
    print("Chessboard calibration: SPACE=grab, q=finish (need >= 10 grabs).")

    while True:
        frame = grab(cam)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)

        disp = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            cv2.drawChessboardCorners(disp, CHESSBOARD, corners, found)

        cv2.imshow("calib", disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' ') and found:
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"  grabbed {len(objpoints)}")
        if k == ord('q') and len(objpoints) >= 10:
            break

    cv2.destroyAllWindows()
    ret, K, D, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    print(f"reprojection error = {ret:.4f} px")
    np.save("camera_matrix.npy", K)
    np.save("dist_coeffs.npy", D)
    cam.stop()
    return K, D


def calibrate_laser_plane(K, D):
    cam = make_camera(still=False)
    all_pts_3d = []
    depths_used = []

    print("\nLaser plane calibration.")
    print("For each depth: put a FLAT white board at a measured distance,")
    print("enter the distance in mm, aim the laser at the board, press ENTER.")
    print(f"Use at least {MIN_LASER_DEPTHS} depths (3-5 recommended).")
    print("Blank line to finish.")

    while True:
        d_str = input("Depth in mm (blank to finish): ").strip()
        if not d_str:
            if len(all_pts_3d) < MIN_LASER_DEPTHS:
                print(f"  need at least {MIN_LASER_DEPTHS} depths "
                      f"({len(all_pts_3d)} so far) -- keep going")
                continue
            break
        try:
            depth_mm = float(d_str)
        except ValueError:
            print("  not a number, try again")
            continue

        input("  aim laser at board, press ENTER to capture...")
        frame = grab(cam)
        pts_uv = detect_laser_line_red(frame)
        if len(pts_uv) < 20:
            print(f"  only {len(pts_uv)} laser points -- adjust threshold or aim")
            continue

        pts = pts_uv.reshape(-1, 1, 2).astype(np.float32)
        und = cv2.undistortPoints(pts, K, D)
        x_n = und[:, 0, 0]
        y_n = und[:, 0, 1]

        Z = depth_mm
        X = x_n * Z
        Y = y_n * Z
        all_pts_3d.append(np.stack([X, Y, np.full_like(X, Z)], axis=1))
        depths_used.append(Z)
        print(f"  collected {len(X)} points at Z={Z:.1f} mm "
              f"({len(all_pts_3d)} depths so far)")

    cam.stop()

    if len(all_pts_3d) < MIN_LASER_DEPTHS:
        print(f"ERROR: need at least {MIN_LASER_DEPTHS} depths to fit a reliable plane")
        return None

    P = np.vstack(all_pts_3d)
    centroid = P.mean(axis=0)
    centered = P - centroid
    _, _, Vt = np.linalg.svd(centered)
    normal = Vt[-1]
    d = -normal.dot(centroid)
    plane = np.append(normal, d).astype(np.float32)

    rms_residual_mm = float(np.sqrt(np.mean((centered @ normal) ** 2)))

    np.save("laser_plane.npy", plane)
    print("laser plane =", plane)
    print(f"depths used: {[f'{z:.1f}' for z in depths_used]} mm")
    print(f"plane fit RMS residual = {rms_residual_mm:.3f} mm "
          "(should be a small fraction of a mm -- if large, re-run with a "
          "flatter board or more/better-separated depths)")
    return plane


def cmd_calibrate(args):
    if os.path.exists("camera_matrix.npy") and os.path.exists("dist_coeffs.npy"):
        K = np.load("camera_matrix.npy")
        D = np.load("dist_coeffs.npy")
        print("loaded existing intrinsics")
    else:
        K, D = calibrate_camera()
    calibrate_laser_plane(K, D)


# ============================================================
# Mode: scan
# ============================================================

def send_cloud(sock, P):
    """4-byte little-endian length prefix + raw float32 payload."""
    if P.shape[0] == 0:
        # Nothing sent for an empty capture -- the viewer only treats an
        # explicit 0-count header as end-of-scan, so skipping the send
        # here avoids colliding with the real end-of-scan sentinel.
        return
    header = struct.pack("<I", P.shape[0])
    sock.sendall(header + P.astype(np.float32).tobytes())


def cmd_scan(args):
    for f in ("camera_matrix.npy", "dist_coeffs.npy", "laser_plane.npy"):
        if not os.path.exists(f):
            print(f"ERROR: {f} not found -- run `calibrate` first")
            return
    K     = np.load("camera_matrix.npy")
    D     = np.load("dist_coeffs.npy")
    PLANE = np.load("laser_plane.npy")
    print("calibration loaded")

    cam = make_camera(still=True)
    ser = open_esp_serial(args.port)

    if not wait_for_ready(ser):
        print(f"WARNING: no READY banner within {READY_TIMEOUT_S:.0f}s -- "
              "continuing anyway, but command/reply sync is not guaranteed.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.laptop_ip, LAPTOP_PORT))
    print("connected to viewer")

    scan_ok = False
    try:
        print(esp_send(ser, f"L{SCAN_MIN_DEG},{SCAN_MAX_DEG}"))
        print(esp_send(ser, f"S{ESP_SPEED}"))
        print(esp_send(ser, f"A{ESP_ACCEL}"))
        print(esp_send(ser, f"C{ESP_TOL_DEG},{ESP_MAX_ATT}"))

        angles = np.arange(SCAN_MIN_DEG, SCAN_MAX_DEG + 1e-9, SCAN_STEP_DEG)
        n = len(angles)
        t0 = time.time()

        for i, ang in enumerate(angles):
            esp_send(ser, f"T{ang:.4f}")
            if not esp_wait_settled(ser):
                print(f"[{i+1}/{n}] ang={ang:.3f}  SKIPPED")
                continue

            frame = grab(cam)
            pts_uv = detect_laser_line_red(frame)
            P = triangulate(pts_uv, K, D, PLANE)
            Pw = rotate_about_scan_axis(P, ang)
            send_cloud(sock, Pw)

            elapsed = time.time() - t0
            print(f"[{i+1}/{n}] ang={ang:+.3f}  pts={P.shape[0]:4d}  t={elapsed:.1f}s")

        scan_ok = True

    except Exception:
        print("Scan aborted by exception:")
        traceback.print_exc()

    finally:
        # Whatever happened above, always try to bring the scan head to a
        # safe stop and release every resource. Each step is independently
        # guarded so one failed cleanup step doesn't block the rest.
        for step in (
            lambda: esp_send(ser, "X"),
            lambda: sock.sendall(struct.pack("<I", 0)),  # end-of-scan sentinel
            sock.close,
            cam.stop,
            ser.close,
        ):
            try:
                step()
            except Exception:
                pass

    print("scan complete" if scan_ok else "scan aborted")


# ============================================================
# CLI
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="650nm line-laser scanner -- Pi side (check / calibrate / scan)")
    parser.add_argument("mode", choices=["check", "calibrate", "scan"])
    parser.add_argument("--port", default=ESP_PORT,
                         help=f"ESP32 serial port (default: {ESP_PORT})")
    parser.add_argument("--laptop-ip", default=LAPTOP_IP,
                         help=f"viewer.py host IP (default: {LAPTOP_IP})")
    args = parser.parse_args()

    if args.mode == "check":
        cmd_check(args)
    elif args.mode == "calibrate":
        cmd_calibrate(args)
    elif args.mode == "scan":
        cmd_scan(args)


if __name__ == "__main__":
    main()
