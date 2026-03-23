# ros2_wifi_bridge

유선 NIC(DDS Domain 15)에서 수신한 ROS2 토픽을 WiFi NIC(DDS Domain 0)으로 재발행하는 멀티프로세스 브리지.

## 구조

```
robot_network_bridge/
├── robot_network_bridge/
│   └── wifi_bridge.py       # 브리지 본체
└── config/
    ├── wifi_bridge.yaml      # 메인 설정
    ├── cyclonedds_wired.xml  # 유선 NIC CycloneDDS 프로파일
    └── cyclonedds_wifi.xml   # WiFi NIC CycloneDDS 프로파일
```

## 동작 방식

```
[유선 NIC: enP2p1s0]          [WiFi NIC: wlP1p1s0]
 Domain 15                      Domain 0
 subscriber_process  ──Pipe──>  publisher_process
```

- **subscriber_process**: 유선 NIC(`CYCLONEDDS_URI=wired.xml`, `ROS_DOMAIN_ID=15`)에서 토픽 구독 → `multiprocessing.Pipe`로 직렬화된 메시지 전송
- **publisher_process**: WiFi NIC(`CYCLONEDDS_URI=wifi.xml`, `ROS_DOMAIN_ID=0`)에서 메시지 수신 → 재발행

두 프로세스가 각각 독립된 `CYCLONEDDS_URI`/`ROS_DOMAIN_ID` 환경변수를 가지기 때문에 NIC 격리가 완전히 보장됨.

### `/tf_static` 특수 처리

여러 노드가 `/tf_static`에 각자 발행하므로, 수신할 때마다 `(frame_id, child_frame_id)` 키로 transform을 누적(upsert)한 뒤 전체를 재발행한다. TRANSIENT_LOCAL 구독자(RViz, rqt 등)가 모든 static transform을 한번에 받을 수 있도록 하기 위함.

## 실행

```bash
# 설정파일 자동 참조 (config/wifi_bridge.yaml)
python3 robot_network_bridge/wifi_bridge.py

# 설정파일 직접 지정
python3 robot_network_bridge/wifi_bridge.py config/wifi_bridge.yaml
```

## 설정 (`config/wifi_bridge.yaml`)

```yaml
wired_xml: /path/to/cyclonedds_wired.xml
wifi_xml:  /path/to/cyclonedds_wifi.xml
source_domain_id: 15   # 유선 쪽 ROS_DOMAIN_ID
target_domain_id: 0    # WiFi 쪽 ROS_DOMAIN_ID

topics:
  /tf:
    type: tf2_msgs/msg/TFMessage
    reliable: true
  /tf_static:
    type: tf2_msgs/msg/TFMessage
    latched: true
    reliable: true
```

### 토픽 QoS 옵션

| 옵션 | subscriber | publisher |
|------|-----------|-----------|
| `latched: true` | RELIABLE + TRANSIENT_LOCAL, depth=100 | RELIABLE + TRANSIENT_LOCAL, depth=1 |
| `reliable: true` | RELIABLE + VOLATILE, depth=100 | RELIABLE + VOLATILE, depth=100 |
| (기본값) | BEST_EFFORT + VOLATILE, depth=10 | BEST_EFFORT + VOLATILE, depth=10 |

## CycloneDDS 설정

| 파일 | NIC |
|------|-----|
| `cyclonedds_wired.xml` | `enP2p1s0` (유선) |
| `cyclonedds_wifi.xml` | `wlP1p1s0` (WiFi) |

NIC 이름이 다르면 각 XML 파일의 `NetworkInterface name` 값만 수정.

## 의존성

```bash
pip install pyyaml
# ROS2 Humble 이상 + tf2_msgs 패키지 필요
```
