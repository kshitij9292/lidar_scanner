# LiDAR Line-Scanner -- Full Package

650 nm line-laser scanner: NEMA23 + TB6600 + AS5600 pan head (ESP32),
Pi camera capturing the laser stripe, laptop showing the live point cloud.

```
lidar_scanner/
├── firmware/
│   ├── scan_head_firmware.ino     # flash to the ESP32 (pan-head controller)
│   └── as5600_wiring_test.ino     # flash FIRST, standalone AS5600 sanity check
├── pi/
│   └── lidar_scan.py              # copy to the Pi (check / calibrate / scan)
└── laptop/
    └── viewer.py                  # run on the laptop (live point-cloud viewer)
```

---

## 1. ESP32 firmware

**Flash `as5600_wiring_test.ino` first**, before the real firmware. It's a
throwaway sanity check: open Serial Monitor at 115200 and confirm it finds
the AS5600 at `0x36`, reports "Magnet detected, strength OK", and the
angle tracks smoothly as you turn the shaft by hand. Only move on once
that's clean -- it isolates wiring/magnet problems from anything in the
real closed-loop firmware.

If upload fails with `Wrong boot mode (0x13)`: hold **BOOT**, tap
**EN/RST**, keep holding BOOT until you see `Connecting...` in the
console, then release. If that fixes it, your board's auto-reset circuit
isn't toggling -- close any other program with the COM port open
(Serial Monitor, PuTTY, a second IDE window) before future uploads.

**Then flash `scan_head_firmware.ino`** -- this is the real pan-head
controller: closed-loop AS5600 position control, travel limits, and a
`Z` command to auto-detect and correct motor/encoder direction (see
below). Wiring is documented in the header comment of the file itself.

**First-time bring-up on the real firmware**, over Serial Monitor at
115200:
```
L-20,20        # set travel limits (output-shaft degrees) -- required before any M/T
Z              # auto-detect motor/encoder direction, once per hardware config
               #   -> OK,DIR_DETECT,ALREADY_CORRECT   or
               #   -> OK,DIR_DETECT,INVERTED_CORRECTED (saved to flash, done)
?              # confirm status -- check dir_inverted, limits_set
T0             # move to output-shaft 0 deg and confirm it settles
```
`Z` only needs to be run once ever per physical build (it's saved to
flash and reloaded on every boot) -- re-run it only if you rewire or
swap the motor/encoder.

Full serial command reference is in the header comment of
`scan_head_firmware.ino`.

---

## 2. Raspberry Pi setup (fresh image)

```bash
sudo apt update && sudo apt full-upgrade -y && sudo reboot
# after reboot:
sudo apt install -y python3-opencv python3-numpy python3-serial
sudo usermod -aG dialout $USER      # then log out and back in
rpicam-hello --list-cameras          # confirm the camera is detected
```

Copy `pi/lidar_scan.py` to its own directory on the Pi (e.g. `~/scanner/`).
Find the ESP32's port: `ls /dev/ttyUSB* /dev/ttyACM*`.

Full setup instructions (Lite-image extras, wiring recap, permissions)
are also in the header docstring of `lidar_scan.py` itself.

---

## 3. Run order, every time

```bash
# 1. Laptop -- start the viewer FIRST, it listens for the Pi to connect
python viewer.py

# 2. Pi -- quick hardware check
python3 lidar_scan.py check --port /dev/ttyUSB0

# 3. Pi -- once per physical setup (skip if camera_matrix.npy / 
#    dist_coeffs.npy / laser_plane.npy already exist and nothing moved)
python3 lidar_scan.py calibrate

# 4. Pi -- every scan
python3 lidar_scan.py scan --port /dev/ttyUSB0 --laptop-ip <laptop's IP>
```

Re-run `calibrate` if the camera, laser, or their relative mounting
changes at all. Re-run firmware's `Z` command only if the motor or
encoder wiring changes.

---

## Tuning quick-reference

| Symptom | Fix |
|---|---|
| Laser line breaks up / disappears | lower `RED_THRESHOLD` in `lidar_scan.py` |
| Background speckle detected as laser | raise `RED_THRESHOLD` |
| Scan too slow | raise `SCAN_STEP_DEG` (fewer bins) or `ESP_SPEED`/`ESP_ACCEL` |
| Scanned object shears/twists across angle | wrong `SCAN_ROTATION_AXIS_CAMERA_FRAME` in `lidar_scan.py` -- see the comment above it |
| `ERR,DIR_DETECT,NO_MOTION` on `Z` | encoder not coupled to shaft, or AS5600 not seeing the magnet -- re-run `as5600_wiring_test.ino` |
| `ERR,LIMITS_NOT_SET` | send `L<min>,<max>` before any `M`/`T` |
