import json
from pathlib import Path
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String

from .map_io import cloud_message, load_map, map_identity, obstacle_sources


class OfflineMapServer(Node):
    def __init__(self):
        super().__init__('offline_map_server')
        directory = self.declare_parameter('map_directory', '').value
        database = self.declare_parameter('database_directory', '').value
        self.metadata, self.support, self.obstacles = load_map(directory, database)
        self.identity = map_identity(self.metadata, self.support, self.obstacles)
        self.identity.update(
            session_id=uuid.uuid4().hex,
            publisher_node=self.get_fully_qualified_name(),
            map_directory=str(Path(directory).expanduser().resolve()),
            database_directory=str(Path(database).expanduser().resolve()))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.support_pub = self.create_publisher(PointCloud2, '/offline_map/support', qos)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/offline_map/obstacles', qos)
        self.identity_pub = self.create_publisher(String, '/offline_map/identity', qos)
        self.id_pub = self.create_publisher(String, '/offline_map/id', qos)
        self.ready_pub = self.create_publisher(Bool, '/offline_map/ready', qos)
        self.published = False
        self.timer = self.create_timer(0.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Loaded fixed map: {len(self.support)} support, {len(self.obstacles)} static hazards; '
            f'map_id={self.identity["map_id"]}; directory={self.identity["map_directory"]}')

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            return
        if not self.published:
            self.identity['publication_stamp'] = dict(sec=stamp.sec, nanosec=stamp.nanosec)
            self.identity_pub.publish(String(data=json.dumps(
                self.identity, sort_keys=True, separators=(',', ':'))))
            self.id_pub.publish(String(data=self.identity['map_id']))
            self.support_pub.publish(cloud_message(self.support, stamp))
            self.obstacle_pub.publish(cloud_message(self.obstacles, stamp,
                sources=obstacle_sources(self.metadata, self.obstacles)))
            self.published = True
        self.ready_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = None
    try:
        node = OfflineMapServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
