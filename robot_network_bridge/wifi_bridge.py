#!/usr/bin/env python3
"""
WiFi Bridge - bidirectional.

유선(wired, domain N, NIC A) <-> WiFi(domain M, NIC B) 양방향 토픽 브리지.

direction 별 동작:
  - wired_to_wifi : 유선쪽 sub -> WiFi쪽 pub
  - wifi_to_wired : WiFi쪽 sub -> 유선쪽 pub
  - bidirectional : 양쪽 sub+pub. 자기가 publish한 메시지가 같은 도메인의
                    자기 sub로 돌아오는 echo 를 막기 위해, publish 직전
                    md5(payload) 를 TTL 캐시에 적재. sub 콜백에서 동일
                    해시가 캐시에 있으면 drop 후 캐시 엔트리 consume.

dedup 한계:
  - 동일 payload가 TTL(2s) 내에 정상적으로 두 번 발생하면 두 번째가 drop됨.
    명령류 토픽에 한해 사용 권장.

tf_static 특수 처리:
  - 여러 노드가 /tf_static 에 각자 publish 하므로, publish 측에서
    (frame_id, child_frame_id) 키로 upsert 한 누적본을 전체 재발행.
    TRANSIENT_LOCAL 구독자(RViz 등)가 모든 static transform 을 한번에
    받을 수 있도록 하기 위함.
"""
import os
import sys
import time
import yaml
import hashlib
import multiprocessing
import importlib
import signal

TF_STATIC_TOPIC = '/tf_static'
DEDUP_TTL = 2.0  # seconds, bidirectional echo 방지 캐시 TTL
PIPE_POLL_PERIOD = 0.01  # 10ms

DIR_W2W  = 'wired_to_wifi'
DIR_W2WR = 'wifi_to_wired'
DIR_BIDI = 'bidirectional'
VALID_DIRECTIONS = {DIR_W2W, DIR_W2WR, DIR_BIDI}


def resolve_msg_type(type_str):
    parts = type_str.split('/')
    module = importlib.import_module(f'{parts[0]}.{parts[1]}')
    return getattr(module, parts[2])


def _make_sub_qos(latched, reliable, QoSProfile, ReliabilityPolicy, DurabilityPolicy):
    if latched:
        return QoSProfile(
            depth=100,  # tf_static 처럼 여러 노드가 각자 발행할 수 있음
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    if reliable:
        return QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _make_pub_qos(latched, reliable, QoSProfile, ReliabilityPolicy, DurabilityPolicy):
    if latched:
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    if reliable:
        return QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def bridge_side_process(
    side_name,        # 'wired' or 'wifi' (로그/노드명)
    cyclonedds_uri,   # 이쪽 NIC 의 CycloneDDS XML 경로
    domain_id,        # 이쪽 ROS_DOMAIN_ID
    sub_topics,       # dict: topic_name -> topic_info  (이쪽이 구독)
    pub_topics,       # dict: topic_name -> topic_info  (이쪽이 발행)
    bidi_topics,      # set : 양방향 토픽 (echo dedup 대상)
    out_pipe,         # 이쪽 sub 수신분을 반대쪽에게 전달
    in_pipe,          # 반대쪽 sub 가 보낸 것을 받아서 이쪽에 publish
):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    os.environ['CYCLONEDDS_URI'] = cyclonedds_uri
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message, deserialize_message

    rclpy.init()
    node = Node(f'wifi_bridge_{side_name}')

    # bidi 토픽에 한해, 이 프로세스가 직전에 publish 한 (hash, ts) 들을 기록.
    # 같은 도메인의 자기 sub 로 echo 가 돌아오면 drop.
    dedup_cache = {t: [] for t in bidi_topics}

    def _evict(topic):
        now = time.monotonic()
        dedup_cache[topic] = [
            (h, t) for h, t in dedup_cache[topic] if now - t < DEDUP_TTL
        ]

    # ── Publishers ─────────────────────────────────────────────────
    publishers = {}
    for topic_name, info in pub_topics.items():
        msg_type = resolve_msg_type(info['type'])
        latched  = info.get('latched', False)
        reliable = info.get('reliable', False)
        qos = _make_pub_qos(latched, reliable,
                            QoSProfile, ReliabilityPolicy, DurabilityPolicy)
        pub = node.create_publisher(msg_type, topic_name, qos)
        publishers[topic_name] = (pub, msg_type)
        node.get_logger().info(
            f'[{side_name} PUB] {topic_name} [{info["type"]}] '
            f'latched={latched} reliable={reliable} '
            f'bidi={topic_name in bidi_topics}'
        )

    # ── Subscribers ────────────────────────────────────────────────
    def make_sub_cb(t_name, t_type_str):
        is_bidi = t_name in bidi_topics

        def cb(msg):
            try:
                data = serialize_message(msg)
            except Exception:
                return
            if is_bidi:
                _evict(t_name)
                h = hashlib.md5(data).digest()
                cache = dedup_cache[t_name]
                for i, (eh, _ts) in enumerate(cache):
                    if eh == h:
                        cache.pop(i)  # consume one matching entry
                        node.get_logger().info(
                            f'[{side_name} RX-ECHO drop] {t_name} {len(data)}B',
                            throttle_duration_sec=2.0,
                        )
                        return
            node.get_logger().info(
                f'[{side_name} RX->pipe] {t_name} {len(data)}B',
                throttle_duration_sec=2.0,
            )
            try:
                out_pipe.send((t_name, t_type_str, bytes(data)))
            except Exception:
                pass

        return cb

    for topic_name, info in sub_topics.items():
        msg_type = resolve_msg_type(info['type'])
        latched  = info.get('latched', False)
        reliable = info.get('reliable', False)
        qos = _make_sub_qos(latched, reliable,
                            QoSProfile, ReliabilityPolicy, DurabilityPolicy)
        node.create_subscription(
            msg_type, topic_name,
            make_sub_cb(topic_name, info['type']),
            qos,
        )
        node.get_logger().info(
            f'[{side_name} SUB] {topic_name} [{info["type"]}] '
            f'latched={latched} reliable={reliable} '
            f'bidi={topic_name in bidi_topics}'
        )

    # /tf_static 누적 (이 프로세스가 /tf_static 의 publisher 일 때만 사용)
    tf_static_cache = {}

    def poll_pipe():
        try:
            while in_pipe.poll(0):
                topic_name, _type_str, data = in_pipe.recv()
                if topic_name not in publishers:
                    continue
                pub, msg_type = publishers[topic_name]
                msg = deserialize_message(data, msg_type)

                if topic_name == TF_STATIC_TOPIC:
                    before = len(tf_static_cache)
                    for tr in msg.transforms:
                        key = (tr.header.frame_id, tr.child_frame_id)
                        tf_static_cache[key] = tr
                    if len(tf_static_cache) > before:
                        merged = msg.__class__()
                        merged.transforms = list(tf_static_cache.values())
                        # bidi 가능성은 없지만 일관성 위해 캐시 기록
                        if topic_name in bidi_topics:
                            _evict(topic_name)
                            dedup_cache[topic_name].append(
                                (hashlib.md5(serialize_message(merged)).digest(),
                                 time.monotonic())
                            )
                        pub.publish(merged)
                        node.get_logger().info(
                            f'[{side_name} tf_static] merged total='
                            f'{len(tf_static_cache)}'
                        )
                    continue

                if topic_name in bidi_topics:
                    _evict(topic_name)
                    dedup_cache[topic_name].append(
                        (hashlib.md5(data).digest(), time.monotonic())
                    )
                node.get_logger().info(
                    f'[{side_name} pipe->TX] {topic_name} {len(data)}B',
                    throttle_duration_sec=2.0,
                )
                pub.publish(msg)
        except (EOFError, BrokenPipeError):
            pass
        except Exception:
            pass

    node.create_timer(PIPE_POLL_PERIOD, poll_pipe)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    if len(sys.argv) >= 2:
        config_path = sys.argv[1]
    else:
        config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', 'config', 'wifi_bridge.yaml'
        )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    config_dir = os.path.dirname(os.path.abspath(config_path))

    topics_config = config.get('topics', {})
    wired_xml = os.path.join(config_dir, config.get('wired_xml', ''))
    wifi_xml  = os.path.join(config_dir, config.get('wifi_xml', ''))

    # 신/구 키 모두 허용
    wired_domain = config.get('wired_domain_id',
                              config.get('source_domain_id', 15))
    wifi_domain  = config.get('wifi_domain_id',
                              config.get('target_domain_id', 0))

    if wired_domain == wifi_domain:
        print(f'ERROR: wired_domain ({wired_domain}) == wifi_domain ({wifi_domain})')
        print('Multicast feedback loop. Use different ROS_DOMAIN_IDs.')
        sys.exit(1)

    if not topics_config:
        print('No topics configured')
        sys.exit(1)

    wired_sub_topics = {}   # 유선쪽이 구독 = (w2w + bidi)
    wired_pub_topics = {}   # 유선쪽이 발행 = (w2wr + bidi)
    wifi_sub_topics  = {}   # WiFi쪽이 구독 = (w2wr + bidi)
    wifi_pub_topics  = {}   # WiFi쪽이 발행 = (w2w + bidi)
    bidi_topics = set()

    for name, info in topics_config.items():
        direction = info.get('direction', DIR_W2W)
        if direction not in VALID_DIRECTIONS:
            print(f'ERROR: unknown direction "{direction}" for topic {name}')
            sys.exit(1)
        if direction in (DIR_W2W, DIR_BIDI):
            wired_sub_topics[name] = info
            wifi_pub_topics[name]  = info
        if direction in (DIR_W2WR, DIR_BIDI):
            wifi_sub_topics[name]  = info
            wired_pub_topics[name] = info
        if direction == DIR_BIDI:
            bidi_topics.add(name)

    print(f'[bridge] wired={wired_domain} ({wired_xml})')
    print(f'[bridge] wifi ={wifi_domain} ({wifi_xml})')
    print(f'[bridge] wired_to_wifi: '
          f'{[k for k,v in topics_config.items() if v.get("direction", DIR_W2W)==DIR_W2W]}')
    print(f'[bridge] wifi_to_wired: '
          f'{[k for k,v in topics_config.items() if v.get("direction")==DIR_W2WR]}')
    print(f'[bridge] bidirectional: {sorted(bidi_topics)}')

    # Pipe 2개 (방향별)
    # wired sub -> wifi pub
    w2w_recv, w2w_send = multiprocessing.Pipe(duplex=False)
    # wifi sub -> wired pub
    w2wr_recv, w2wr_send = multiprocessing.Pipe(duplex=False)

    wired_proc = multiprocessing.Process(
        target=bridge_side_process,
        args=(
            'wired', wired_xml, wired_domain,
            wired_sub_topics, wired_pub_topics, bidi_topics,
            w2w_send,    # 이쪽 sub 출력 -> wifi 쪽이 받음
            w2wr_recv,   # wifi 쪽이 sub 한 거 받아서 여기서 publish
        ),
    )
    wifi_proc = multiprocessing.Process(
        target=bridge_side_process,
        args=(
            'wifi', wifi_xml, wifi_domain,
            wifi_sub_topics, wifi_pub_topics, bidi_topics,
            w2wr_send,   # 이쪽 sub 출력 -> wired 쪽이 받음
            w2w_recv,    # wired 쪽이 sub 한 거 받아서 여기서 publish
        ),
    )

    wired_proc.start()
    wifi_proc.start()

    def shutdown(_sig, _frame):
        wired_proc.terminate()
        wifi_proc.terminate()
        wired_proc.join(timeout=3)
        wifi_proc.join(timeout=3)
        if wired_proc.is_alive():
            wired_proc.kill()
        if wifi_proc.is_alive():
            wifi_proc.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    wired_proc.join()
    wifi_proc.join()


if __name__ == '__main__':
    main()
