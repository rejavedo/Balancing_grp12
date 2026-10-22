# TASK 11: Lidar validation diagnostics; TASK 01: sensor interface checks.
"""Read-only scan audit. Does not command motors or change guard parameters."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .core import scan_quality


class Audit(Node):
    def __init__(self):
        super().__init__('scan_audit')
        self.declare_parameter('samples',30)
        self.declare_parameter('output','/tmp/scan_audit.json')
        self.frames=[]; self.started=time.monotonic(); self.done=False
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
    def scan(self,msg):
        q=scan_quality(msg.ranges,msg.range_min,msg.range_max,msg.angle_increment)
        q.update(frame_id=msg.header.frame_id,range_min=msg.range_min,range_max=msg.range_max,
                 angle_increment=msg.angle_increment,
                 stamp_age_s=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9)
        self.frames.append(q)
        if len(self.frames)>=self.get_parameter('samples').value: self.done=True
    def save(self):
        output=Path(self.get_parameter('output').value).expanduser()
        output.parent.mkdir(parents=True,exist_ok=True)
        result={'received_samples':len(self.frames),'frames':self.frames,
                'note':'Inspect invalid fraction/gaps and freshness. Do not treat zero/NaN as free space.'}
        output.write_text(json.dumps(result,indent=2,allow_nan=False))
        self.get_logger().info(f'Saved {len(self.frames)} scans to {output}')


def main():
    rclpy.init(); n=Audit()
    try:
        while rclpy.ok() and not n.done and time.monotonic()-n.started < 15.:
            rclpy.spin_once(n,timeout_sec=.1)
        n.save()
        if not n.done: n.get_logger().error('Timed out before requested sample count; check scan/network')
    finally: n.destroy_node(); rclpy.shutdown()
