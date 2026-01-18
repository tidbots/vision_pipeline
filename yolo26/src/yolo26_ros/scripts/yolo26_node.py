#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
import ultralytics
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


# ---------------- IoU / NMS ----------------

def box_iou_xyxy(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return inter / (area_a + area_b - inter + 1e-9)


def nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> List[int]:
    if len(boxes) == 0:
        return []
    idxs = np.argsort(scores)[::-1]
    keep = []
    while len(idxs) > 0:
        i = int(idxs[0])
        keep.append(i)
        if len(idxs) == 1:
            break
        rest = idxs[1:]
        ious = np.array([box_iou_xyxy(boxes[i], boxes[int(j)]) for j in rest], dtype=np.float32)
        idxs = rest[ious < iou_thr]
    return keep


# ---------------- Tiling ----------------

def compute_tiles(x0: int, y0: int, w: int, h: int, tile: int, overlap: float):
    if tile <= 0:
        return [(0, x0, y0, w, h)]
    step = max(1, int(tile * (1.0 - overlap)))
    tiles = []
    tid = 0
    for y in range(y0, y0 + h, step):
        for x in range(x0, x0 + w, step):
            tw = min(tile, x0 + w - x)
            th = min(tile, y0 + h - y)
            tiles.append((tid, x, y, tw, th))
            tid += 1
            if x + tile >= x0 + w:
                break
        if y + tile >= y0 + h:
            break
    return tiles


# ---------------- Depth ROI ----------------

def depth_to_meters(depth: np.ndarray, depth_scale: float) -> np.ndarray:
    if depth is None:
        return depth
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * float(depth_scale)
    if depth.dtype in (np.float32, np.float64):
        return depth.astype(np.float32)
    return depth.astype(np.float32)


def compute_depth_roi(
    depth_m: np.ndarray,
    min_depth: float,
    max_depth: float,
    roi_margin_px: int,
    min_area_ratio: float,
) -> Optional[Tuple[int, int, int, int]]:
    if depth_m is None or depth_m.size == 0:
        return None

    H, W = depth_m.shape[:2]
    finite = np.isfinite(depth_m)
    mask = finite & (depth_m > min_depth) & (depth_m < max_depth)

    if not np.any(mask):
        return None

    m = (mask.astype(np.uint8) * 255)

    k = max(3, int(min(H, W) * 0.01) | 1)  # odd
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=2)

    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n <= 1:
        return None

    best = None
    best_area = 0
    for lab in range(1, n):
        x, y, w, h, area = stats[lab]
        if int(area) > best_area:
            best_area = int(area)
            best = (int(x), int(y), int(w), int(h))

    if best is None:
        return None

    area_ratio = float(best_area) / float(H * W)
    if area_ratio < float(min_area_ratio):
        return None

    x, y, w, h = best
    x1 = max(0, x - roi_margin_px)
    y1 = max(0, y - roi_margin_px)
    x2 = min(W, x + w + roi_margin_px)
    y2 = min(H, y + h + roi_margin_px)

    if x2 - x1 < 10 or y2 - y1 < 10:
        return None

    return (x1, y1, x2, y2)


# ---------------- Confidence hist image ----------------

def draw_conf_hist_image(scores: List[float], width=360, height=240, bins=12, title="confidence") -> np.ndarray:
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.rectangle(img, (0, 0), (width - 1, height - 1), (255, 255, 255), 1)
    if len(scores) == 0:
        cv2.putText(img, "no scores", (10, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        return img

    s = np.clip(np.array(scores, dtype=np.float32), 0.0, 1.0)
    hist, _ = np.histogram(s, bins=bins, range=(0.0, 1.0))
    hist = hist.astype(np.float32)

    pad = 30
    x0, y0 = pad, pad
    x1, y1 = width - pad, height - pad
    cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), 1)

    maxv = float(np.max(hist)) if np.max(hist) > 0 else 1.0
    bw = max(1, (x1 - x0) // bins)

    for i in range(bins):
        h = int((hist[i] / maxv) * (y1 - y0))
        px1 = x0 + i * bw
        px2 = min(x1, px1 + bw - 1)
        py1 = y1
        py2 = y1 - h
        cv2.rectangle(img, (px1, py2), (px2, py1), (0, 255, 255), -1)

    mean = float(np.mean(s))
    p10 = float(np.percentile(s, 10))
    p50 = float(np.percentile(s, 50))
    p90 = float(np.percentile(s, 90))

    cv2.putText(img, title, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(img, f"n={len(scores)} mean={mean:.2f} p10={p10:.2f} p50={p50:.2f} p90={p90:.2f}",
                (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img


class Yolo26Node:
    def __init__(self):
        rospy.init_node("yolo26_node")

        # Topics
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_preprocessed")
        self.det_topic = rospy.get_param("~detections_topic", "/yolo26/detections")
        self.ann_topic = rospy.get_param("~annotated_topic", "/yolo26/image_annotated")

        self.debug_enable = bool(rospy.get_param("~debug_enable", True))
        self.debug_topic = rospy.get_param("~debug_topic", "/yolo26/image_debug")

        self.hist_enable = bool(rospy.get_param("~hist_enable", True))
        self.hist_topic = rospy.get_param("~hist_topic", "/yolo26/confidence_hist")
        self.hist_window_scores = int(rospy.get_param("~hist_window_scores", 1200))  # scores (not frames)
        self.hist_publish_every = int(rospy.get_param("~hist_publish_every", 10))    # frames

        # Model params
        self.model_path = rospy.get_param("~model_path", "/models/yolo26s.pt")
        self.device = str(rospy.get_param("~device", "0"))  # "0" or "cpu"
        self.imgsz = int(rospy.get_param("~imgsz", 960))
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.iou = float(rospy.get_param("~iou", 0.45))

        # Tiling
        self.tile_enable = bool(rospy.get_param("~tile_enable", True))
        self.tile_size = int(rospy.get_param("~tile_size", 640))
        self.tile_overlap = float(rospy.get_param("~tile_overlap", 0.25))

        # Depth ROI
        self.use_depth_roi = bool(rospy.get_param("~use_depth_roi", False))
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))  # for 16UC1 (mm->m)
        self.min_depth = float(rospy.get_param("~min_depth", 0.25))
        self.max_depth = float(rospy.get_param("~max_depth", 1.80))
        self.roi_margin_px = int(rospy.get_param("~roi_margin_px", 10))
        self.roi_min_area_ratio = float(rospy.get_param("~roi_min_area_ratio", 0.02))
        self.roi_fallback_full = bool(rospy.get_param("~roi_fallback_full", True))

        # Class names YAML
        self.show_class_names = bool(rospy.get_param("~show_class_names", True))
        self.classes_yaml = rospy.get_param("~classes_yaml", "")
        self.class_names: Dict[int, str] = {}

        # -------- Auto re-tune (YOLO side) --------
        # This reacts to lighting changes indirectly (confidence drop / det drop),
        # and gently adjusts conf and overlap within bounds.
        self.auto_tune_enable = bool(rospy.get_param("~auto_tune_enable", False))
        self.auto_tune_every = int(rospy.get_param("~auto_tune_every", 10))  # frames
        self.auto_min_interval = float(rospy.get_param("~auto_tune_min_interval", 0.5))  # sec
        self.auto_ema_alpha = float(rospy.get_param("~auto_ema_alpha", 0.2))

        self.conf_min = float(rospy.get_param("~conf_min", 0.12))
        self.conf_max = float(rospy.get_param("~conf_max", 0.45))
        self.conf_step_down = float(rospy.get_param("~conf_step_down", 0.02))
        self.conf_step_up = float(rospy.get_param("~conf_step_up", 0.01))

        self.overlap_min = float(rospy.get_param("~overlap_min", 0.15))
        self.overlap_max = float(rospy.get_param("~overlap_max", 0.35))
        self.overlap_step_up = float(rospy.get_param("~overlap_step_up", 0.05))
        self.overlap_step_down = float(rospy.get_param("~overlap_step_down", 0.03))

        self.bad_mean_conf_thr = float(rospy.get_param("~bad_mean_conf_thr", 0.35))
        self.bad_zero_det_frames = int(rospy.get_param("~bad_zero_det_frames", 8))
        self.good_mean_conf_thr = float(rospy.get_param("~good_mean_conf_thr", 0.60))
        self.good_det_thr = int(rospy.get_param("~good_det_thr", 2))

        self.bridge = CvBridge()
        self.latest_depth_m: Optional[np.ndarray] = None

        self.conf_scores = deque(maxlen=max(2000, self.hist_window_scores))
        self.frame_count = 0
        self._last_time = time.time()
        self._last_auto_t = 0.0

        # EMA metrics for auto-tune
        self._ema_mean_conf = 0.0
        self._ema_det = 0.0
        self._zero_det_run = 0

        # Load class names
        self._load_class_names()

        # Fail fast: avoid Ultralytics auto-download to RO mount
        if not os.path.exists(self.model_path):
            rospy.logerr("Model weights not found: %s", self.model_path)
            raise RuntimeError(f"weights not found: {self.model_path}")

        rospy.loginfo("Loading YOLO model: %s", self.model_path)
        self.model = ultralytics.YOLO(self.model_path)

        # Publishers
        self.pub_det = rospy.Publisher(self.det_topic, Detection2DArray, queue_size=1)
        self.pub_ann = rospy.Publisher(self.ann_topic, Image, queue_size=1)
        if self.debug_enable:
            self.pub_dbg = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        if self.hist_enable:
            self.pub_hist = rospy.Publisher(self.hist_topic, Image, queue_size=1)

        # Subscribers (after model)
        self.sub_img = rospy.Subscriber(self.input_topic, Image, self.cb, queue_size=1)
        if self.use_depth_roi:
            self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        rospy.loginfo("yolo26_node ready (auto_tune=%s)", self.auto_tune_enable)

    def _load_class_names(self):
        self.class_names = {}
        if not self.show_class_names or not self.classes_yaml:
            return
        try:
            if not os.path.exists(self.classes_yaml):
                rospy.logwarn("classes_yaml not found: %s", self.classes_yaml)
                return
            with open(self.classes_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
            for k, v in data.items():
                try:
                    self.class_names[int(k)] = str(v)
                except Exception:
                    continue
        except Exception as e:
            rospy.logwarn("Failed to load classes_yaml: %s (%s)", self.classes_yaml, e)

    def _cls_label(self, cid: int) -> str:
        if self.show_class_names and cid in self.class_names:
            return self.class_names[cid]
        return str(cid)

    def depth_cb(self, msg: Image):
        try:
            # Try common encodings
            if msg.encoding in ("32FC1", "32FC"):
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            elif msg.encoding in ("16UC1", "mono16"):
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            else:
                # fallback
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.latest_depth_m = depth_to_meters(d, self.depth_scale)
        except Exception:
            self.latest_depth_m = None

    def infer(self, img_bgr: np.ndarray):
        r0 = self.model.predict(
            source=img_bgr,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )[0]

        if r0.boxes is None or len(r0.boxes) == 0:
            return [], [], []

        boxes = r0.boxes.xyxy.cpu().numpy()
        scores = r0.boxes.conf.cpu().numpy()
        cls_ids = r0.boxes.cls.cpu().numpy().astype(int)
        return boxes, scores, cls_ids

    def _auto_tune(self, det_count: int, scores: List[float]):
        """
        Gentle auto re-tune:
          - If confidence drops / zero detections, reduce conf threshold and increase overlap a bit.
          - If stable high confidence and detections exist, relax back (raise conf, lower overlap).
        """
        if not self.auto_tune_enable:
            return

        now = time.time()
        if (self.frame_count % max(1, self.auto_tune_every)) != 0:
            return
        if (now - self._last_auto_t) < self.auto_min_interval:
            return

        mean_conf = float(np.mean(scores)) if len(scores) > 0 else 0.0

        a = self.auto_ema_alpha
        self._ema_mean_conf = (1 - a) * self._ema_mean_conf + a * mean_conf
        self._ema_det = (1 - a) * self._ema_det + a * float(det_count)

        if det_count == 0:
            self._zero_det_run += 1
        else:
            self._zero_det_run = 0

        changed = False

        # Bad condition: low confidence or consecutive zero detections
        bad = (self._ema_mean_conf < self.bad_mean_conf_thr) or (self._zero_det_run >= self.bad_zero_det_frames)

        # Good condition: stable and confident
        good = (self._ema_mean_conf > self.good_mean_conf_thr) and (self._ema_det >= float(self.good_det_thr))

        if bad:
            # Make detection easier + more coverage
            new_conf = max(self.conf_min, self.conf - self.conf_step_down)
            new_ov = min(self.overlap_max, self.tile_overlap + self.overlap_step_up) if self.tile_enable else self.tile_overlap
            if abs(new_conf - self.conf) > 1e-6:
                self.conf = new_conf
                changed = True
            if abs(new_ov - self.tile_overlap) > 1e-6:
                self.tile_overlap = new_ov
                changed = True

            # Depth ROI safety: if ROI is enabled and may be too strict, relax slightly
            if self.use_depth_roi:
                # reduce min area ratio a bit (down to 0.01)
                new_area = max(0.01, self.roi_min_area_ratio * 0.8)
                if abs(new_area - self.roi_min_area_ratio) > 1e-6:
                    self.roi_min_area_ratio = new_area
                    changed = True

        elif good:
            # Return toward precision / speed
            new_conf = min(self.conf_max, self.conf + self.conf_step_up)
            new_ov = max(self.overlap_min, self.tile_overlap - self.overlap_step_down) if self.tile_enable else self.tile_overlap
            if abs(new_conf - self.conf) > 1e-6:
                self.conf = new_conf
                changed = True
            if abs(new_ov - self.tile_overlap) > 1e-6:
                self.tile_overlap = new_ov
                changed = True

        if changed:
            self._last_auto_t = now
            rospy.loginfo_throttle(
                1.0,
                "auto_tune yolo: conf=%.2f overlap=%.2f ema_conf=%.2f ema_det=%.2f zero_run=%d roi_area=%.3f",
                self.conf, self.tile_overlap, self._ema_mean_conf, self._ema_det, self._zero_det_run, self.roi_min_area_ratio
            )

    def cb(self, msg: Image):
        self.frame_count += 1
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        H, W = img.shape[:2]

        # Depth ROI
        roi = None
        if self.use_depth_roi and self.latest_depth_m is not None:
            roi = compute_depth_roi(
                self.latest_depth_m,
                self.min_depth,
                self.max_depth,
                self.roi_margin_px,
                self.roi_min_area_ratio,
            )

        if roi is None and self.use_depth_roi and not self.roi_fallback_full:
            final_boxes, final_scores, final_cls = [], [], []
            tiles = []
            roi_used = None
        else:
            roi_used = roi if roi is not None else (0, 0, W, H)
            rx1, ry1, rx2, ry2 = roi_used
            rw, rh = (rx2 - rx1), (ry2 - ry1)

            tiles = compute_tiles(rx1, ry1, rw, rh, self.tile_size if self.tile_enable else -1, self.tile_overlap)

            all_boxes, all_scores, all_cls = [], [], []
            for tid, x, y, tw, th in tiles:
                crop = img[y:y + th, x:x + tw]
                boxes, scores, cls_ids = self.infer(crop)
                for b, s, c in zip(boxes, scores, cls_ids):
                    x1, y1, x2, y2 = b
                    all_boxes.append([x1 + x, y1 + y, x2 + x, y2 + y])
                    all_scores.append(float(s))
                    all_cls.append(int(c))

            # NMS
            final_boxes, final_scores, final_cls = [], [], []
            if all_boxes:
                all_boxes = np.array(all_boxes, dtype=np.float32)
                all_scores = np.array(all_scores, dtype=np.float32)
                all_cls = np.array(all_cls, dtype=np.int32)
                for cid in np.unique(all_cls):
                    idx = np.where(all_cls == cid)[0]
                    keep = nms_xyxy(all_boxes[idx], all_scores[idx], self.iou)
                    for k in keep:
                        final_boxes.append(all_boxes[idx][k])
                        final_scores.append(float(all_scores[idx][k]))
                        final_cls.append(int(cid))

        # Publish detections
        det_arr = Detection2DArray()
        det_arr.header = msg.header
        for b, s, c in zip(final_boxes, final_scores, final_cls):
            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.x = float((b[0] + b[2]) / 2.0)
            det.bbox.center.y = float((b[1] + b[3]) / 2.0)
            det.bbox.size_x = float(max(0.0, b[2] - b[0]))
            det.bbox.size_y = float(max(0.0, b[3] - b[1]))

            hyp = ObjectHypothesisWithPose()
            hyp.id = int(c)
            hyp.score = float(s)
            det.results.append(hyp)
            det_arr.detections.append(det)

        self.pub_det.publish(det_arr)

        # Annotated image
        ann = img.copy()
        for b, s, c in zip(final_boxes, final_scores, final_cls):
            x1, y1, x2, y2 = map(int, b)
            cv2.rectangle(ann, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = self._cls_label(int(c))
            cv2.putText(ann, f"{label}:{float(s):.2f}",
                        (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)

        ann_msg = self.bridge.cv2_to_imgmsg(ann, "bgr8")
        ann_msg.header = msg.header
        self.pub_ann.publish(ann_msg)

        # Debug image
        now = time.time()
        fps = 1.0 / max(1e-6, now - self._last_time)
        self._last_time = now

        if self.debug_enable:
            dbg = ann.copy()

            if self.use_depth_roi:
                if roi_used is not None:
                    rx1, ry1, rx2, ry2 = roi_used
                    cv2.rectangle(dbg, (rx1, ry1), (rx2 - 1, ry2 - 1), (0, 128, 255), 2)
                    cv2.putText(dbg, f"ROI depth [{self.min_depth:.2f},{self.max_depth:.2f}]m area>= {self.roi_min_area_ratio:.3f}",
                                (rx1 + 5, max(0, ry1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 2, cv2.LINE_AA)
                else:
                    cv2.putText(dbg, "ROI: none", (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 128, 255), 2, cv2.LINE_AA)

            for tid, x, y, tw, th in tiles:
                cv2.rectangle(dbg, (x, y), (x + tw - 1, y + th - 1), (255, 0, 0), 1)

            cv2.putText(dbg, f"FPS:{fps:.1f} tiles:{len(tiles)} det:{len(final_boxes)} imgsz:{self.imgsz} conf:{self.conf:.2f} ov:{self.tile_overlap:.2f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)

            if self.auto_tune_enable:
                cv2.putText(dbg, "AUTO_TUNE:ON", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
            dbg_msg.header = msg.header
            self.pub_dbg.publish(dbg_msg)

        # Confidence histogram
        if self.hist_enable:
            if final_scores:
                for s in final_scores:
                    self.conf_scores.append(float(s))

            if (self.frame_count % max(1, self.hist_publish_every)) == 0:
                recent = list(self.conf_scores)[-min(len(self.conf_scores), self.hist_window_scores):]
                hist_img = draw_conf_hist_image(recent, width=360, height=240, bins=12, title="confidence (recent)")
                hist_msg = self.bridge.cv2_to_imgmsg(hist_img, "bgr8")
                hist_msg.header = msg.header
                self.pub_hist.publish(hist_msg)

        # ---- Auto tune (YOLO side) ----
        self._auto_tune(det_count=len(final_boxes), scores=final_scores)


if __name__ == "__main__":
    Yolo26Node()
    rospy.spin()
