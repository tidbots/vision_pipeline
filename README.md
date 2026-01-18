# TIDBots vision_pipeline

## ToDo
### Depth付き ROI 自動生成
- /camera/depth/image_raw（sensor_msgs/Image）を購読
- **有効深度範囲（min_depth–max_depth）**だけを残す
- Depth マスクから 床・棚候補 ROI を自動生成
- ROI 内だけで YOLO 推論（＝高速＋誤検出低減）

「床の上の小物」「棚の中の物体」に 非常に効く

### class名表示（YAML読み込み）
- classes.yaml を読み込み
- id → class名 に変換
- annotated / debug 画像に class名表示
- detection msg の ObjectHypothesis.id は 従来通り整数（安全）

### confidence ヒストグラム表示（失敗検出）
- 推論 confidence を蓄積
- 直近 N フレーム分の confidence 分布をヒストグラム化
- /yolo26/confidence_hist（Image）として publish
- 「見えてるのに拾えてない」状況を 視覚的に検出

## 全体構成
```
vision_pipeline/
├── compose.yaml
│ 
├── image_preprocessed/
│   ├── docker/
│   │     ├── Dockerfile
│   │     └── entrypoint.sh
│   └─── src/image_preprocessed
│         ├── CMakeLists.txt
│         ├── package.xml
│         ├── launch/
│         │    ├── 
│         │    └── 
│         └── scripts/
│              ├── 
│              └──
│   
└── yolo26
    ├── docker/
    │     ├── Dockerfile
    │     └── entrypoint.sh
    └─── src/yolo26
          ├── CMakeLists.txt
          ├── package.xml
          ├── launch/
          │    ├── 
          │    └── 
          ├── scripts/
          │    ├── 
          │    └──
          └── models/
               └── yolo26s.pt

```
## 流れ
```
/usb_cam/image_raw   (sensor_msgs/Image)
        ↓
[ preprocess_node ]
  - 明るさ推定
  - 自動Gamma補正
  - CLAHE（局所コントラスト補正）
        ↓
/camera/image_preprocessed   (sensor_msgs/Image)
        ↓
[ yolo26_node ]
  - YOLO26 推論
        ↓
/yolo26/detections           (vision_msgs/Detection2DArray)
/yolo26/image_annotated      (sensor_msgs/Image)
```

