# TIDBots vision_pipeline
このリポジトリは、照明変動に強く・小物に強い 物体認識パイプラインを提供します。

- ROS1 Noetic
- Docker / docker compose
- YOLO26（Ultralytics）
- 画像前処理（Gamma + CLAHE）
- Depth を用いた ROI 自動生成
- Tile 推論による小物検出強化
- class 名表示（YAML）
- confidence ヒストグラムによる失敗検出

## 全体構成
```
camera (RGB)
   │
   ▼
image_preprocess container
   ├─ Gamma correction
   ├─ CLAHE
   └─ /camera/image_preprocessed
           │
           ▼
yolo26 container (GPU)
   ├─ Depth ROI 自動生成（optional）
   ├─ Tile 推論
   ├─ YOLO26 推論
   ├─ class名表示
   └─ confidence ヒストグラム
```

## ディレクトリ構成
```
vision_pipeline/
├── compose.yaml
├── READMEW.md
├── image_preprocessed/
│   ├── docker/
│   │     ├── Dockerfile
│   │     └── entrypoint.sh
│   └─── src/image_preprocessed
│         ├── CMakeLists.txt
│         ├── package.xml
│         ├── launch/
│         │    ├── preprocess.launch
│         │    └── 
│         └── scripts/
│              ├── preprocess_node.py
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
          │    ├── yolo26.launch
          │    └── yolo26_usb.launch
          ├── scripts/
          │    ├── yolo26_node.py
          │    └──
          ├── config/
          │   └── classes.yaml
          └── models/
               └── yolo26s.pt
```

## 必要要件
### ハードウェア
- NVIDIA GPU（推奨）
- RGB カメラ
- Depth カメラ（ROI 使用時）

### ソフトウェア
- Ubuntu 20.04
- Docker
- docker compose
- NVIDIA Container Toolkit
- ROS1 Noetic

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
/camera/depth/image_raw   ← 深度画像
        ↓
[ yolo26_node ]
  - YOLO26 推論
        ↓
/yolo26/detections           (vision_msgs/Detection2DArray)
/yolo26/image_annotated      (sensor_msgs/Image)
/yolo26/image_debug
/yolo26/confidence_hist 
```
### Depth付き ROI 自動生成
- /camera/depth/image_raw（sensor_msgs/Image）を購読
- 有効深度範囲（min_depth–max_depth）だけを残す
  - 深度範囲（min_depth ～ max_depth）から ROI を推定
- Depth マスクから 床・棚候補 ROI を自動生成
  - 床・棚以外を排除
- ROI 内だけで YOLO 推論（＝高速＋誤検出低減）
  - 誤検出削減 & 高速化

「床の上の小物」「棚の中の物体」に 非常に効く

### Tile 推論
- ROI 内を分割推論
- 小物（ペットボトル・カップ等）に強い

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


## 使い方
```
git clone https://github.com/tidbots/vision_pipeline.git
cd vision_pipeline
docker compose build
docker compose up
```

```
cd vision_pipeline
docker compose exec yolo26 bash
rqt_image_view
```





