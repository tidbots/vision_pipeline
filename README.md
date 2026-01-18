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

## パラメータチューニング指針（会場照明別）
対象ノード：
- image_preprocess（Gamma / CLAHE）
- yolo26_node（imgsz / conf / tile / ROI）

### 0. チューニングの基本方針（重要）
1. まず前処理（画像の見え）を安定させる
2. 次に YOLO の conf / tile を調整
3. 最後に Depth ROI を詰める

👉 いきなり YOLO 側を触らないのがコツ

### 1.照明パターン別・推奨設定
#### A. 暗い会場（夕方・照度不足・影が強い）
症状
- 全体が暗い
- 小物が背景に溶ける
- confidence が全体的に低い

image_preprocess
```
gamma: 1.3 〜 1.6
clahe_clip: 3.0
clahe_grid: 8
```
yolo26
```
conf: 0.15
imgsz: 960
tile_size: 640
tile_overlap: 0.30
```

Depth ROI
```
min_depth: 0.25
max_depth: 1.50
roi_margin_px: 15
```

✅ ポイント
- 暗い会場では gamma ↑ が最優先
- conf は必ず下げる（失敗検出は histogram で）

#### B. 明るすぎる会場（白飛び・直射照明）
症状
- 白い床・テーブルが飛ぶ
- ハイライトで物体輪郭が消える

image_preprocess
```
gamma: 0.75 〜 0.9
clahe_clip: 1.5
clahe_grid: 8
```

yolo26
```
conf: 0.25
imgsz: 960
tile_size: 640
tile_overlap: 0.25
```

Depth ROI
```
min_depth: 0.30
max_depth: 1.80
```

✅ ポイント
- gamma < 1.0 で白飛び抑制
- CLAHE を強くしすぎない（ノイズ化する）

#### C. ムラのある照明（スポットライト・影あり）
症状
- 場所によって明るさが違う
- 同じ物体が認識されたりされなかったり

image_preprocess
```
gamma: auto（暗→1.3 / 明→0.85）
clahe_clip: 2.5
clahe_grid: 8
```
※ auto は平均輝度で切り替え（実装済みなら有効）

yolo26
```
conf: 0.20
imgsz: 960
tile_size: 640
tile_overlap: 0.30
```

Depth ROI
```
roi_fallback_full: true
roi_min_area_ratio: 0.015
```

✅ ポイント
- Tile overlap を増やす
- ROI が不安定なら full fallback を許可

#### D. 理想的な会場（均一・十分な照度）
症状
- 全体が見やすい
- 認識は安定

image_preprocess
```
gamma: 1.0
clahe_clip: 2.0
clahe_grid: 8
```

yolo26
```
conf: 0.30
imgsz: 960
tile_size: 640
tile_overlap: 0.20
```

Depth ROI
```
min_depth: 0.30
max_depth: 2.00
```

✅ ポイント
- 無理にいじらない
- conf を上げて誤検出を減らす

### 2. confidence ヒストグラムの見方（超重要）
/yolo26/confidence_hist を必ず確認してください。

良い状態
- 分布が 0.6〜0.9 に山
- p50 > 0.5

危険信号 🚨
- 分布が 0.2〜0.4 に集中
- フレームごとに大きく揺れる

👉 対策：
- conf ↓
- gamma 再調整
- tile_overlap ↑

### 3. 会場入り後の「5分チューニング手順」
#### ① 画像を見る
```
rqt_image_view /camera/image_preprocessed
```
- 暗い → gamma ↑
- 白飛び → gamma ↓

#### ② debug 画像を見る
```
rqt_image_view /yolo26/image_debug
```

- ROI が変 → depth 範囲修正
- tile 足りない → overlap ↑

#### ③ histogram を見る
```
rqt_image_view /yolo26/confidence_hist
```
- 全体低い → conf ↓
- ノイズ多い → conf ↑ or CLAHE ↓

### 4. 鉄板プリセット
万能スタート設定（迷ったらこれ）
```
gamma: 1.1
clahe_clip: 2.5
conf: 0.20
imgsz: 960
tile_size: 640
tile_overlap: 0.30
min_depth: 0.30
max_depth: 1.80
```

### 5. やってはいけないこと ❌
- conf を 0.4 以上に固定
- ROI を厳しくしすぎる
- histogram を見ない

