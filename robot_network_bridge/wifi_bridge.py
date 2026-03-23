#!/usr/bin/env python3
"""
WiFi Bridge: subscribes to topics on wired interface and republishes on WiFi.
Uses two separate processes with a Pipe for 1:1 message passing.

tf_static 특수 처리:
  - 여러 노드가 /tf_static 에 각자 publish하므로, 수신할 때마다
    transform을 누적(frame_id+child_frame_id 기준 upsert)한 뒤
    전체를 다시 publish한다.
  - 이렇게 해야 TRANSIENT_LOCAL 구독자(RViz, rqt 등)가
    모든 static transform을 한번에 받을 수 있다.
"""
import os
import sys
import yaml
import multiprocessing
import importlib
import signal

TF_STATIC_TOPIC = '/tf_static'


def resolve_msg_type(type_str):
    parts = type_str.split('/')
    module = importlib.import_module(f'{parts[0]}.{parts[1]}')
    return getattr(module, parts[2])


def subscriber_process(topics_config, conn, wired_xml, source_domain_id):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    os.environ['CYCLONEDDS_URI'] = wired_xml
    os.environ['ROS_DOMAIN_ID'] = str(source_domain_id)

    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import serialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init()
    node = Node('wifi_bridge_sub')

    for topic_name, topic_info in topics_config.items():
        msg_type = resolve_msg_type(topic_info['type'])

        def make_callback(t_name, t_type_str):
            def callback(msg):
                data = serialize_message(msg)
                try:
                    conn.send((t_name, t_type_str, bytes(data)))
                except Exception:
                    pass
            return callback

        latched  = topic_info.get('latched', False)
        reliable = topic_info.get('reliable', False)

        if latched:
            sub_qos = QoSProfile(
                depth=100,   # 여러 노드가 tf_static을 각자 발행하므로 충분히 크게
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        elif reliable:
            sub_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            sub_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

        node.create_subscription(
            msg_type, topic_name,
            make_callback(topic_name, topic_info['type']),
            sub_qos
        )
        node.get_logger().info(
            f'[SUB] {topic_name} [{topic_info["type"]}] '
            f'latched={latched} reliable={reliable}'
        )

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def publisher_process(topics_config, conn, wifi_xml, target_domain_id):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    os.environ['CYCLONEDDS_URI'] = wifi_xml
    os.environ['ROS_DOMAIN_ID'] = str(target_domain_id)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message

    rclpy.init()
    node = Node('wifi_bridge_pub')

    qos_best_effort = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    qos_reliable = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    qos_latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    publishers = {}
    for topic_name, topic_info in topics_config.items():
        msg_type = resolve_msg_type(topic_info['type'])
        latched  = topic_info.get('latched', False)
        reliable = topic_info.get('reliable', False)

        if latched:
            pub_qos = qos_latched
        elif reliable:
            pub_qos = qos_reliable
        else:
            pub_qos = qos_best_effort

        pub = node.create_publisher(msg_type, topic_name, pub_qos)
        publishers[topic_name] = (pub, msg_type)
        node.get_logger().info(
            f'[PUB] {topic_name} [{topic_info["type"]}] '
            f'-> domain {target_domain_id} via WiFi '
            f'latched={latched} reliable={reliable}'
        )

    # /tf_static 누적 저장소
    # key: (frame_id, child_frame_id) → TransformStamped
    tf_static_cache = {}

    running = True
    while running:
        try:
            if conn.poll(0.1):
                topic_name, type_str, data = conn.recv()
                if topic_name not in publishers:
                    continue

                pub, msg_type = publishers[topic_name]
                msg = deserialize_message(data, msg_type)

                # /tf_static 특수 처리: 누적 후 전체 재발행
                if topic_name == TF_STATIC_TOPIC:
                    before = len(tf_static_cache)
                    for transform in msg.transforms:
                        key = (transform.header.frame_id,
                               transform.child_frame_id)
                        tf_static_cache[key] = transform  # 항상 upsert

                    # 새 transform이 추가됐을 때만 재발행
                    if len(tf_static_cache) > before:
                        merged_msg = msg.__class__()
                        merged_msg.transforms = list(tf_static_cache.values())
                        pub.publish(merged_msg)
                        node.get_logger().info(
                            f'[tf_static] merged total={len(tf_static_cache)} transforms'
                        )
                else:
                    pub.publish(msg)

        except (EOFError, BrokenPipeError):
            running = False
        except Exception:
            pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    if len(sys.argv) >= 2:
        config_path = sys.argv[1]
    else:
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'wifi_bridge.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    topics_config    = config.get('topics', {})
    wired_xml        = config.get('wired_xml', '')
    wifi_xml         = config.get('wifi_xml', '')
    source_domain_id = config.get('source_domain_id', 0)
    target_domain_id = config.get('target_domain_id', 0)

    if source_domain_id == target_domain_id:
        print(f'WARNING: source_domain_id ({source_domain_id}) == target_domain_id ({target_domain_id})')
        print('This WILL cause a multicast feedback loop. Use different domain IDs.')
        sys.exit(1)

    if not topics_config:
        print('No topics configured')
        sys.exit(1)

    parent_conn, child_conn = multiprocessing.Pipe()

    sub_proc = multiprocessing.Process(
        target=subscriber_process,
        args=(topics_config, child_conn, wired_xml, source_domain_id),
    )
    pub_proc = multiprocessing.Process(
        target=publisher_process,
        args=(topics_config, parent_conn, wifi_xml, target_domain_id),
    )

    sub_proc.start()
    pub_proc.start()

    def shutdown(sig, frame):
        sub_proc.terminate()
        pub_proc.terminate()
        sub_proc.join(timeout=3)
        pub_proc.join(timeout=3)
        if sub_proc.is_alive():
            sub_proc.kill()
        if pub_proc.is_alive():
            pub_proc.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    sub_proc.join()
    pub_proc.join()


if __name__ == '__main__':
    main()