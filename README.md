# KETI Jetson Robot

NVIDIA Jetson Orin Nano 기반 자율주행 로봇 플랫폼

## 시스템 구성

```
┌─────────────────────────────────────────────────────────────┐
│                    Web Interface (8888)                      │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│  │ Camera  │  │  LiDAR  │  │Joystick │  │  Mode   │        │
│  │  16:9   │  │  View   │  │ Control │  │ Select  │        │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘        │
└───────┼────────────┼────────────┼────────────┼──────────────┘
        │            │            │            │
        ▼            ▼            ▼            ▼
┌───────────────────────────────────────────────────────────┐
│                   ROS2 Humble                              │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐     │
│  │ camera   │ │ rosbridge│ │  robot   │ │   mode   │     │
│  │  _node   │ │ websocket│ │  driver  │ │controller│     │
│  │  (8080)  │ │  (9090)  │ │          │ │          │     │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘     │
└───────────────────────────────────────────────────────────┘
        │                          │
        ▼                          ▼
┌───────────────┐          ┌───────────────┐
│  CSI Camera   │          │   LD19 LiDAR  │
│  (IMX219)     │          │   /dev/ttyUSB0│
└───────────────┘          └───────────────┘
```

## 포트 구성

| Port | Service | Description |
|------|---------|-------------|
| 8888 | nginx | Web UI (HTML/CSS/JS) |
| 8080 | camera_node | MJPEG 카메라 스트림 |
| 9090 | rosbridge | WebSocket (ROS2 통신) |

## 동작 모드

### Control 탭
| Mode | Icon | 설명 |
|------|------|------|
| **Manual** | 🎮 | 조이스틱으로 직접 조작 |
| **Obstacle** | 🚧 | 장애물 감지 시 자동 정지 |

### Mode 탭
| Mode | Icon | 설명 | 스크립트 |
|------|------|------|----------|
| **Patrol** | 🔄 | 정해진 경로 순찰 | patrol.py |
| **Follower** | 👤 | 사람 감지 및 추종 | person_follower.py |
| **Lane** | 🛣️ | 차선 인식 주행 | lane_detector.py, lane_follower.py |
| **YOLO** | 🎯 | 객체 감지 (YOLOv8) | yolo_detector.py |
| **Parking** | 🅿️ | 자동 주차 | auto_parking.py |

### Map 탭
| Mode | Icon | 설명 | 필요 패키지 |
|------|------|------|------------|
| **SLAM** | 🗺️ | 실시간 지도 생성 | robot_slam (Cartographer) |
| **Nav** | 🧭 | 자율 주행 | robot_navigation (Nav2) |

## 카메라 파이프라인

```
CSI Camera (IMX219)
       │
       ▼
nvarguscamerasrc (Jetson 하드웨어)
       │
       ▼
nvvidconv (flip-method=2, 180° 회전)
       │
       ▼
nvjpegenc (하드웨어 JPEG 인코딩, quality=60)
       │
       ├──────────────────┐
       ▼                  ▼
 ROS2 Topic          HTTP MJPEG
 /camera/image_raw   http://IP:8080/stream
 /compressed
```

### 카메라 설정
- 해상도: **640x360** (16:9)
- FPS: **15**
- 품질: **60**
- Flip: **2** (180° 회전)

## 조이스틱 제어

### 속도 제어
- **SMOOTH = 0.10**: 부드러운 가/감속
- **MAX_LIN = 0.5 m/s**: 최대 직진 속도
- **MAX_ANG = 2.0 rad/s**: 최대 회전 속도
- **Speed Slider**: 10% ~ 150% 속도 조절

### 키보드 제어
| Key | Action |
|-----|--------|
| W | 전진 |
| S | 후진 |
| A | 좌회전 |
| D | 우회전 |
| Space | 정지 |

## 설치 및 실행

### 1. 환경 설정
```bash
export KETI_MODEL=robot
export LDS_MODEL=LDS-04
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. 서비스 시작
```bash
# systemd 서비스로 자동 시작
sudo systemctl start robot.service

# 또는 수동 실행
ros2 launch robot_bringup robot.launch.py
```

### 3. 웹 접속
```
http://192.168.10.1:8888/
```

## 패키지 구조

```
ros2_ws/src/
├── robot_bringup/      # 로봇 시작 launch 파일
├── robot_driver/       # 모터 드라이버 (Modbus)
├── robot_description/  # URDF 모델
├── robot_web/          # 웹 인터페이스 + 카메라 노드
├── robot_ai/           # AI 모드 (YOLO, Follower, Patrol 등)
├── robot_slam/         # SLAM (Cartographer)
├── robot_navigation/   # 자율 주행 (Nav2)
├── robot_teleop/       # 원격 조작
├── robot_msgs/         # 커스텀 메시지
├── rf2o_laser_odometry/# LiDAR 기반 오도메트리
└── ldlidar_stl_ros2/   # LD19 LiDAR 드라이버
```

## 하드웨어

- **보드**: NVIDIA Jetson Orin Nano (8GB)
- **카메라**: IMX219 CSI Camera
- **LiDAR**: LD19 (360° 2D)
- **모터**: Modbus 통신 (ttyTHS1)

## 트러블슈팅

### 카메라 안 보임
```bash
# argus 데몬 확인
sudo systemctl status nvargus-daemon

# 카메라 테스트
gst-launch-1.0 nvarguscamerasrc ! nvvidconv ! xvimagesink
```

### 웹 연결 안 됨
```bash
# rosbridge 확인
lsof -i :9090

# nginx 확인
sudo systemctl status nginx
```

### LiDAR 데이터 없음
```bash
# 권한 확인
sudo chmod 666 /dev/ttyUSB0

# 토픽 확인
ros2 topic hz /scan
```

## nginx 설정

```bash
# /etc/nginx/sites-available/robot
server {
    listen 8888 default_server;
    root /var/www/robot;
    index index.html;
    server_name _;

    location / {
        try_files $uri $uri/ =404;
    }

    # Rosbridge WebSocket proxy
    location /rosbridge {
        proxy_pass http://127.0.0.1:9090;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_read_timeout 86400;
    }

    add_header Access-Control-Allow-Origin *;
}
```

```bash
# 설정 활성화
sudo ln -s /etc/nginx/sites-available/robot /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl restart nginx
sudo systemctl enable nginx
```

## 자동 시작 설정

```bash
# robot.service 활성화
sudo systemctl enable robot.service
sudo systemctl enable nginx

# 확인
sudo systemctl status robot.service
sudo systemctl status nginx
```

## License

Apache 2.0
