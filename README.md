# TIDBots vision_pipeline

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

