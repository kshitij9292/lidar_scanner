"""
viewer.py -- run on the LAPTOP.

Requires (on the laptop):
    pip install open3d numpy
Usage:
    python viewer.py
Then start scanner.py on the Pi.
"""

import socket
import struct
import numpy as np
import open3d as o3d

LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 5005


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((LISTEN_IP, LISTEN_PORT))
    srv.listen(1)
    print(f"listening on {LISTEN_IP}:{LISTEN_PORT}")
    conn, addr = srv.accept()
    print("scanner connected from", addr)

    # Persistent point cloud object. Reuse the SAME object each frame --
    # replacing it breaks Open3D's GL binding.
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3), dtype=np.float64))

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="650nm line-laser scanner",
                      width=1280, height=720)
    vis.add_geometry(cloud)
    vis.get_render_option().point_size = 2.0
    vis.get_render_option().background_color = np.array([0.05, 0.05, 0.08])

    accumulated = []
    first_frame = True

    try:
        while True:
            hdr = recv_exact(conn, 4)
            if hdr is None:
                print("connection closed")
                break
            n = struct.unpack("<I", hdr)[0]
            if n == 0:
                print("end of scan")
                break

            raw = recv_exact(conn, n * 3 * 4)
            if raw is None:
                print("truncated payload")
                break

            P = np.frombuffer(raw, dtype=np.float32).reshape(n, 3)
            accumulated.append(P)

            all_pts = np.vstack(accumulated).astype(np.float64)

            # Colour by height (Z) for a quick visual sanity check.
            zmin, zmax = all_pts[:, 2].min(), all_pts[:, 2].max()
            if zmax - zmin > 1e-6:
                t = (all_pts[:, 2] - zmin) / (zmax - zmin)
            else:
                t = np.zeros_like(all_pts[:, 2])
            colors = np.stack([t,
                               0.3 + 0.4 * (1.0 - t),
                               1.0 - t], axis=1)

            cloud.points = o3d.utility.Vector3dVector(all_pts)
            cloud.colors = o3d.utility.Vector3dVector(colors)

            vis.update_geometry(cloud)
            # Only auto-fit the view on the very first frame -- resetting
            # it on every update fights anyone trying to orbit/zoom while
            # the scan is still running.
            if first_frame:
                vis.reset_bounding_box()
                first_frame = False
            if not vis.poll_events():
                break
            vis.update_renderer()

            print(f"received {n} pts (total {all_pts.shape[0]})")

    finally:
        conn.close()
        srv.close()
        vis.destroy_window()


if __name__ == "__main__":
    main()
