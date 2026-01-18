#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)

import ultralytics


# -----------------------------
# IoU / NMS
# -----------------------------
def box_iou_xyxy(a, b):
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


def nms_xyxy(boxes, scores, iou_thr):
    if len(boxes) == 0:
        return []
    idxs = np.argsort(scores)[::-1]
    keep = []
    while len(idxs) > 0:
        i = idxs[0]
        keep.append(i)
        if len(idxs) == 1:
            break
        rest = idxs[1:]
        ious = np.array([box_iou_xyxy(boxes[i], boxes[j]) for j in rest])
        idxs = rest[ious < iou_thr]
    return keep


# -----------------------------
# Tiling
# -----------------------------
def compute_tiles(w, h, tile, overlap):
    if tile <= 0:
        return [(0, 0, w, h)]
    step = max(1, int(tile * (1.0 - overlap)))
    tiles = []
    tid = 0
    for y in range(0, h, step):
        for x in range(0, w, step):
            tw = min(tile, w - x)
            th = min(tile, h - y)
            tiles.append((tid, x, y, tw, th))
            tid += 1
            if x + tile >= w:
                break
        if y + tile >= h:
            break
    return tiles


# -----------------------------
# Main Node
# -----------------------------
class Yolo26Node:
    def __init__(self):
        rospy.init_node("yolo26_node")

        # --- params ---
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_preprocessed")
        self.det_topic = rospy.get_param("~detections_topic", "~detections")
        self.ann_topic = rospy.get_param("~annotated_topic", "~image_annotated")

        # debug
        self.debug_enable = rospy.get_param("~debug_enable", True)
        self.debug_topic = rospy.get_param("~debug_topic", "~image_debug")

        self.model_path = rospy.get_param("~model_path", "/models/yolo26s.pt")
        self.device = str(rospy.get_param("~device", "0"))
        self.imgsz = int(rospy.get_param("~imgsz", 960))
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.iou = float(rospy.get_param("~iou", 0.45))

        self.tile_enable = bool(rospy.get_param("~tile_enable", True))
        self.tile_size = int(rospy.get_param("~tile_size", 640))
        self.tile_overlap = float(rospy.get_param("~tile_overlap", 0.2))

        self.bridge = CvBridge()

        # --- ROS ---
        self.sub = rospy.Subscriber(self.input_topic, Image, self.cb, queue_size=1)
        self.pub_det = rospy.Publisher(self.det_topic, Detection2DArray, queue_size=1)
        self.pub_ann = rospy.Publisher(self.ann_topic, Image, queue_size=1)
        if self.debug_enable:
            self.pub_dbg = rospy.Publisher(self.debug_topic, Image, queue_size=1)

        # --- model ---
        rospy.loginfo("Loading YOLO model: %s", self.model_path)
        self.model = ultralytics.YOLO(self.model_path)

        self.last_time = time.time()
        rospy.loginfo("yolo26_node ready")

    def infer(self, img):
        r = self.model.predict(
            source=img,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )[0]
        if r.boxes is None:
            return [], [], []
        return (
            r.boxes.xyxy.cpu().numpy(),
            r.boxes.conf.cpu().numpy(),
            r.boxes.cls.cpu().numpy().astype(int),
        )

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        H, W = img.shape[:2]

        tiles = (
            compute_tiles(W, H, self.tile_size, self.tile_overlap)
            if self.tile_enable
            else [(0, 0, 0, W, H)]
        )

        all_boxes, all_scores, all_cls = [], [], []

        for tid, x, y, w, h in tiles:
            crop = img[y : y + h, x : x + w]
            boxes, scores, cls = self.infer(crop)
            for b, s, c in zip(boxes, scores, cls):
                x1, y1, x2, y2 = b
                all_boxes.append([x1 + x, y1 + y, x2 + x, y2 + y])
                all_scores.append(float(s))
                all_cls.append(int(c))

        # --- NMS ---
        final_boxes, final_scores, final_cls = [], [], []
        if all_boxes:
            all_boxes = np.array(all_boxes)
            all_scores = np.array(all_scores)
            all_cls = np.array(all_cls)
            for cid in np.unique(all_cls):
                idx = np.where(all_cls == cid)[0]
                keep = nms_xyxy(all_boxes[idx], all_scores[idx], self.iou)
                for k in keep:
                    final_boxes.append(all_boxes[idx][k])
                    final_scores.append(all_scores[idx][k])
                    final_cls.append(cid)

        # --- publish detections ---
        dets = Detection2DArray()
        dets.header = msg.header
        for b, s, c in zip(final_boxes, final_scores, final_cls):
            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.x = float((b[0] + b[2]) / 2)
            det.bbox.center.y = float((b[1] + b[3]) / 2)
            det.bbox.size_x = float(b[2] - b[0])
            det.bbox.size_y = float(b[3] - b[1])
            hyp = ObjectHypothesisWithPose()
            hyp.id = int(c)
            hyp.score = float(s)
            det.results.append(hyp)
            dets.detections.append(det)
        self.pub_det.publish(dets)

        # --- annotated image ---
        ann = img.copy()
        for b, s, c in zip(final_boxes, final_scores, final_cls):
            x1, y1, x2, y2 = map(int, b)
            cv2.rectangle(ann, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                ann, f"{c}:{s:.2f}", (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
        self.pub_ann.publish(self.bridge.cv2_to_imgmsg(ann, "bgr8"))

        # --- debug image ---
        if self.debug_enable:
            dbg = ann.copy()
            for tid, x, y, w, h in tiles:
                cv2.rectangle(dbg, (x, y), (x + w, y + h), (255, 0, 0), 1)
                cv2.putText(
                    dbg, f"T{tid}", (x + 5, y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1
                )

            fps = 1.0 / max(1e-6, time.time() - self.last_time)
            self.last_time = time.time()
            cv2.putText(
                dbg, f"FPS:{fps:.1f} tiles:{len(tiles)} det:{len(final_boxes)}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))


if __name__ == "__main__":
    Yolo26Node()
    rospy.spin()
