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

## 照明変化に対するパラメータの自動再チューニング
- image_preprocess：自動Gamma/CLAHE（照明変化の主因をここで吸収）
- yolo26：自動 conf / tile_overlap（検出が落ちたときの保険）
の 2段構えで入れる（どちらも launch で個別にON/OFF可）。

### 使い方（ON/OFF の切替）
自動再チューニング ON（推奨）
```
launch 内で auto_tune_enable=true
```

OFF（固定パラメータで運用）
```
launch 内で auto_tune_enable=false
```
例：preprocess.launch のみ OFF にする（YOLO側だけ自動、なども可能）

### 方針
```
camera
  ↓
image_preprocess
  ├─ 輝度統計（mean / std）
  ├─ gamma 自動調整
  └─ clahe 自動調整
        ↓
yolo26
  ├─ confidence 分布監視
  ├─ conf 自動微調整
  ├─ tile_overlap 微調整
  └─ 異常時フラグ
```

### 照明変化の検出（image_preprocess側）
#### ① 監視指標（軽量・確実）
各フレームで以下を計算：

指標	意味
- mean_luma	全体の明るさ
- std_luma	明るさのばらつき
- sat_ratio	白飛び率（>245）
- dark_ratio	黒潰れ率（<10）

#### 照明状態の分類（例）
状態	条件
- DARK	mean < 90
- BRIGHT	mean > 170
- SATURATED	sat_ratio > 0.15
- LOW_CONTRAST	std < 35
- NORMAL	上記以外

※ 10フレーム移動平均で判定（瞬間変化に反応しない）  

### Gamma / CLAHE の自動調整（preprocess）
基本ルール（安全側）
- 暗い → gamma ↑
- 明るい → gamma ↓
- 白飛び → gamma ↓ + clahe_clip ↓
- コントラスト低 → clahe_clip ↑

実際の制御例
```
if state == "DARK":
    gamma = min(gamma + 0.05, 1.6)
elif state == "BRIGHT":
    gamma = max(gamma - 0.05, 0.7)
elif state == "SATURATED":
    gamma = max(gamma - 0.08, 0.75)
    clahe_clip = max(clahe_clip - 0.2, 1.5)
elif state == "LOW_CONTRAST":
    clahe_clip = min(clahe_clip + 0.3, 3.5)
```
⚠️ 1フレームで大きく変えない（±0.05）


### 認識結果を使った自己評価（yolo26側）
#### 監視する指標

指標	意味
- det_count	検出数
- mean_conf	平均 confidence
- p10_conf	下位10%
- hist_shape	分布の歪み

すでに実装済み：
- /yolo26/confidence_hist

### 認識が悪化した時の自動調整（yolo26）
#### 異常検出条件（例）
```
mean_conf < 0.35 が 2秒以上継続
または
det_count == 0 が 10フレーム以上
```

#### 自動調整ルール
① confidence 閾値
```
conf = max(conf - 0.02, 0.12)
```
② tile overlap
```
tile_overlap = min(tile_overlap + 0.05, 0.35)
```
③ Depth ROI 緩和
```
roi_min_area_ratio ↓
roi_fallback_full = True
```
👉 「見えない時ほど広く・甘く」

### フィードバックループ（重要）
```
照明変化
 ↓
preprocess 自動調整
 ↓
yolo confidence 改善？
 ↓
YES → 何もしない
NO  → yolo 側も微調整
```

絶対にやらないこと
- モデル切替
- imgsz 変更
- 再学習
- ノード再起動

### 競技向けフェイルセーフ設計
状態遷移（イメージ）
```
NORMAL
 ↓（照明変化）
ADJUSTING
 ↓（改善）
STABLE
 ↓（失敗）
DEGRADED（conf↓ tile↑ ROI緩和）
```

DEGRADED 状態でも動き続ける
- タスク中に止まらない

