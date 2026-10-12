# LiDAR Line-Scanner: Full Package
 
---
 
- Pan-head gearbox: [kshitij9292/Nema23_planetary_gearbox_25-1-](https://github.com/kshitij9292/Nema23_planetary_gearbox_25-1-) (MIT license)


650 nm line-laser scanner: a NEMA 23 stepper with a 25:1 planetary gearbox, driven by a TB6600 and an ESP32 with AS5600 closed-loop feedback, forms the pan head. A Pi camera captures the laser stripe and a laptop shows the live point cloud.
 
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
 
## System overview
 
| Part | Role |
|------|------|
| NEMA 23 + 25:1 printed planetary gearbox | Pans the laser + camera head |
| TB6600 | Stepper driver |
| AS5600 magnetic encoder | Reads the output-shaft angle for closed-loop control |
| ESP32 | Pan-head controller (serial commands, travel limits, closed loop) |
| Raspberry Pi + camera | Captures the laser stripe, triangulates points |
| Laptop (`viewer.py`) | Live point-cloud display |
 
---
 
## 0. Mechanical: pan-head gearbox
 
The pan head uses the open-source 3D-printed NEMA 23 planetary gearbox:
<https://github.com/kshitij9292/Nema23_planetary_gearbox_25-1->
 
**What it is:** a two-stage 5:1 x 5:1 reduction (25:1 total) with an integrated 8 mm ball-race thrust bearing for axial loads. It mixes printed gears with steel hardware. The repo ships STL files (`Body`, `CAp`, `Outmount1`, `Outmount2`, `planet`, `spider1`, `sun`), a `.3mf`, and a BOM spreadsheet.
 
**Hardware (from the repo BOM):**
 
| Item | Spec | Qty |
|------|------|-----|
| Primary bearings | SKF 608-2ZZ | 2 |
| Planet bearings | SKF 625-2Z | 2 |
| Thrust balls | 8 mm steel | 25 |
| M5x25 SHCS | main housing | 10 |
| M4x16 BHSF | component mounting | 4 |
| M5x25 BHSF | secondary plate | 3 |
| M4 hex nuts | retention | 4 |
 
**Print settings (repo recommendation):** ASA or PETG (especially the sun gear and motor plate, which sit near the warm motor), 6 perimeters, 40% gyroid infill, 0.15 mm layers, horizontal expansion calibrated to about +/-0.02 mm to limit backlash.
 
**Assembly order:** press the sun gear onto the motor shaft (set screw if applicable), assemble the first-stage planets with the 625 bearings, grease the race lightly with lithium grease and seat all 25 balls, stack the second-stage carrier and top housing, then close with the M5 bolts in a star pattern.
 
### Step math (check this against the firmware)
 
The gearbox README quotes 40,000 steps/rev and 0.009 deg/step at "1/16 microstepping". Those figures actually correspond to **1/8** microstepping. For a 1.8 deg motor with the 25:1 gearbox:
 
| TB6600 microstepping | Steps per output rev | Output deg per step |
|----------------------|----------------------|---------------------|
| 1/8 | 40,000 | 0.009 |
| 1/16 | 80,000 | 0.0045 |
 
Make sure the steps-per-degree constant in `scan_head_firmware.ino` matches the DIP-switch setting on your TB6600. A mismatch scales every angle by a constant factor and distorts the point cloud.
 
### AS5600 placement
 
- **Mount the magnet on the gearbox output shaft.** The firmware works in output-shaft degrees, and this placement lets the closed loop see and correct gearbox backlash. Resolution is 12-bit (about 0.088 deg), which is plenty for this application.
- Mounting it on the motor shaft is not recommended: the encoder would wrap every 14.4 deg of output travel, and backlash would go uncorrected.
- Keep the magnet centered on the shaft axis with a 1-3 mm air gap, and verify the gap stays constant through the full sweep (printed housings can flex).
### Reducing backlash effects in scans
 
- Approach every scan bin from the same direction (sweep one way, return to start, repeat).
- Let the head settle before capturing each frame.
- Keep TB6600 current conservative; vibration shows up as laser-line jitter.
---
 
## 1. ESP32 firmware
 
Flash `as5600_wiring_test.ino` first, before the real firmware. It's a throwaway sanity check. Open Serial Monitor at 115200 and confirm it finds the AS5600 at `0x36`, reports "Magnet detected, strength OK", and that the angle tracks smoothly as you turn the shaft by hand. Only move on once that's clean; it isolates wiring and magnet problems from anything in the real closed-loop firmware.
 
**If upload fails with `Wrong boot mode (0x13)`:** hold BOOT, tap EN/RST, keep holding BOOT until you see `Connecting...` in the console, then release. If that fixes it, your board's auto-reset circuit isn't toggling. Close any other program with the COM port open (Serial Monitor, PuTTY, a second IDE window) before future uploads.
 
Then flash `scan_head_firmware.ino`. This is the real pan-head controller: closed-loop AS5600 position control, travel limits, and a `Z` command that auto-detects and corrects motor/encoder direction. Wiring is documented in the header comment of the file itself.
 
### First-time bring-up (Serial Monitor, 115200)
 
```
L-20,20        # set travel limits (output-shaft degrees), required before any M/T
Z              # auto-detect motor/encoder direction, once per hardware config
               #   -> OK,DIR_DETECT,ALREADY_CORRECT   or
               #   -> OK,DIR_DETECT,INVERTED_CORRECTED (saved to flash, done)
?              # confirm status: check dir_inverted, limits_set
T0             # move to output-shaft 0 deg and confirm it settles
```
 
`Z` only needs to be run once per physical build. The result is saved to flash and reloaded on every boot. Re-run it only if you rewire or swap the motor or encoder.
 
The full serial command reference is in the header comment of `scan_head_firmware.ino`.
 
---
 
## 2. Raspberry Pi setup (fresh image)
 
```bash
sudo apt update && sudo apt full-upgrade -y && sudo reboot
# after reboot:
sudo apt install -y python3-opencv python3-numpy python3-serial
sudo usermod -aG dialout $USER      # then log out and back in
rpicam-hello --list-cameras          # confirm the camera is detected
```
 
Copy `pi/lidar_scan.py` to its own directory on the Pi (e.g. `~/scanner/`). Find the ESP32's port with:
 
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
 
Full setup instructions (Lite-image extras, wiring recap, permissions) are also in the header docstring of `lidar_scan.py`.
 
---
 
## 3. Run order (every time)
 
```bash
# 1. Laptop: start the viewer FIRST, it listens for the Pi to connect
python viewer.py
 
# 2. Pi: quick hardware check
python3 lidar_scan.py check --port /dev/ttyUSB0
 
# 3. Pi: once per physical setup (skip if camera_matrix.npy /
#    dist_coeffs.npy / laser_plane.npy already exist and nothing moved)
python3 lidar_scan.py calibrate
 
# 4. Pi: every scan
python3 lidar_scan.py scan --port /dev/ttyUSB0 --laptop-ip <laptop's IP>
```
 
When to redo things:
 
- Re-run `calibrate` if the camera, the laser, or their relative mounting changes at all.
- Re-run the firmware's `Z` command only if the motor or encoder wiring changes.
---
 
## Tuning quick-reference
 
| Symptom | Fix |
|---------|-----|
| Laser line breaks up / disappears | Lower `RED_THRESHOLD` in `lidar_scan.py` |
| Background speckle detected as laser | Raise `RED_THRESHOLD` |
| Scan too slow | Raise `SCAN_STEP_DEG` (fewer bins) or `ESP_SPEED` / `ESP_ACCEL` |
| Scanned object shears/twists across angle | Wrong `SCAN_ROTATION_AXIS_CAMERA_FRAME` in `lidar_scan.py`; see the comment above it |
| `ERR,DIR_DETECT,NO_MOTION` on `Z` | Encoder not coupled to the shaft, or AS5600 not seeing the magnet; re-run `as5600_wiring_test.ino` |
| `ERR,LIMITS_NOT_SET` | Send `L<min>,<max>` before any `M`/`T` |
| All angles scaled by a constant factor (e.g. 2x) | Microstepping DIP switches don't match the firmware's steps-per-degree constant; see Step math above |
| Point cloud jitter or ghosting | Wait for the head to settle before capture; approach each bin from the same direction (gearbox backlash) |
 
 
