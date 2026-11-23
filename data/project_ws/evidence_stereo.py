#!/usr/bin/env python3
"""Stereo disparity evidence (no chessboard required).

Captures N stereo frame pairs from the sim, rectifies them with the
recalibrated yaml, computes SGBM disparity, and reports statistics.
This proves the stereo calibration produces a valid disparity map.
"""
import os, json, time
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

CALIB_DIR = "/root/data/project_ws/calibration_sim"
OUT = "/root/data/project_ws/calibration_sim/evidence"
N_FRAMES = 20
MAX_WAIT_S = 120.0
os.makedirs(OUT, exist_ok=True)


def block(doc, key):
    b = doc[key]
    return np.array(b["data"], dtype=float).reshape(b["rows"], b["cols"])


def load(side):
    with open(os.path.join(CALIB_DIR, f"{side}_sim.yaml")) as f:
        d = yaml.safe_load(f)
    return dict(
        K=block(d, "camera_matrix"),
        D=np.array(d["distortion_coefficients"]["data"], dtype=float),
        R=block(d, "rectification_matrix"),
        P=block(d, "projection_matrix"),
        w=int(d["image_width"]), h=int(d["image_height"]),
    )


L = load("left")
R = load("right")
w, h = L["w"], L["h"]
mapLx, mapLy = cv2.initUndistortRectifyMap(L["K"], L["D"], L["R"], L["P"], (w, h), cv2.CV_16SC2)
mapRx, mapRy = cv2.initUndistortRectifyMap(R["K"], R["D"], R["R"], R["P"], (w, h), cv2.CV_16SC2)

sbm = cv2.StereoSGBM_create(
    minDisparity=0, numDisparities=96, blockSize=9,
    uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
    disp12MaxDiff=1, preFilterCap=63, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

baseline_m = abs(R["P"][0, 3]) / R["P"][0, 0]
print(f"[evidence_stereo] baseline from P2: {baseline_m:.6f} m (physical 0.06028)")
print(f"[evidence_stereo] fx={L['K'][0,0]:.2f} fy={L['K'][1,1]:.2f} "
      f"cx={L['K'][0,2]:.2f} cy={L['K'][1,2]:.2f}")


class StereoEv(Node):
    def __init__(self):
        super().__init__("stereo_evidence")
        self.bridge = CvBridge()
        self.frames = 0
        self.start = time.time()
        self.valid_frac = []
        self.disp_mean = []
        self.disp_median = []
        self.disp_std = []
        subL = message_filters.Subscriber(self, Image, "/stereo/stereo_camera/left/image_raw")
        subR = message_filters.Subscriber(self, Image, "/stereo/stereo_camera/right/image_raw")
        sync = message_filters.ApproximateTimeSynchronizer([subL, subR], 10, slop=0.1)
        sync.registerCallback(self.cb)
        self.create_timer(5.0, self._log)
        self.get_logger().info(f"capturing up to {N_FRAMES} stereo pairs ...")

    def _log(self):
        self.get_logger().info(f"frames={self.frames}/{N_FRAMES} (t={time.time()-self.start:.0f}s)")

    def cb(self, lm, rm):
        if time.time() - self.start > MAX_WAIT_S:
            self._finish()
            return
        if self.frames >= N_FRAMES:
            self._finish()
            return
        try:
            Limg = self.bridge.imgmsg_to_cv2(lm, "bgr8")
            Rimg = self.bridge.imgmsg_to_cv2(rm, "bgr8")
        except Exception:
            return
        gL = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY)
        gR = cv2.cvtColor(Rimg, cv2.COLOR_BGR2GRAY)
        rL = cv2.remap(gL, mapLx, mapLy, cv2.INTER_LINEAR)
        rR = cv2.remap(gR, mapRx, mapRy, cv2.INTER_LINEAR)
        disp = sbm.compute(rL, rR).astype(np.float32) / 16.0
        valid = disp >= 0
        self.valid_frac.append(float(valid.mean()))
        if valid.any():
            v = disp[valid]
            self.disp_mean.append(float(v.mean()))
            self.disp_median.append(float(np.median(v)))
            self.disp_std.append(float(v.std()))
        self.frames += 1
        n = self.frames
        if n <= 2 or n == N_FRAMES:
            d8 = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            dv = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
            cv2.imwrite(os.path.join(OUT, f"stereo_{n:02d}.png"),
                        np.hstack([cv2.cvtColor(rL, cv2.COLOR_GRAY2BGR),
                                   cv2.cvtColor(rR, cv2.COLOR_GRAY2BGR), dv]))
        if n % 5 == 0:
            self.get_logger().info(
                f"  frame {n}: valid={self.valid_frac[-1]:.2%} "
                f"mean={self.disp_mean[-1] if self.disp_mean else 0:.1f}px")
        if self.frames >= N_FRAMES:
            self._finish()

    def _finish(self):
        if getattr(self, "_done", False):
            return
        self._done = True
        vf = np.array(self.valid_frac)
        dm = np.array(self.disp_mean) if self.disp_mean else np.array([0.0])
        dmed = np.array(self.disp_median) if self.disp_median else np.array([0.0])
        ds = np.array(self.disp_std) if self.disp_std else np.array([0.0])
        res = {
            "n_frames": int(self.frames),
            "valid_pixel_frac": {
                "mean": float(vf.mean()), "min": float(vf.min()), "max": float(vf.max()),
            },
            "disparity_px": {
                "mean": float(dm.mean()), "median": float(dmed.mean()), "std": float(ds.mean()),
            },
            "baseline_m": {
                "calibrated": float(baseline_m),
                "physical": 0.06028,
                "ratio": float(baseline_m / 0.06028),
            },
            "intrinsics": {
                "fx": float(L["K"][0, 0]), "fy": float(L["K"][1, 1]),
                "cx": float(L["K"][0, 2]), "cy": float(L["K"][1, 2]),
            },
        }
        with open(os.path.join(OUT, "evidence_stereo.json"), "w") as f:
            json.dump(res, f, indent=2)
        self.get_logger().info("=== STEREO EVIDENCE ===")
        self.get_logger().info(json.dumps(res, indent=2))
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = StereoEv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
