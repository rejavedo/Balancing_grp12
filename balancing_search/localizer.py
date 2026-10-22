# TASK 06: External YOLO detection filtering; TASK 07: map-frame target localization.
"""YOLO floor-contact localization; calibrated raw-image coordinates required."""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from yolo_msgs.msg import DetectionArray
from tf2_ros import Buffer, TransformListener, TransformException
from .core import rotation, ground_hit, Consensus


class TargetLocalizer(Node):
    def __init__(self):
        super().__init__('target_localizer')
        defaults = dict(target_class='CHANGE_ME', confidence=.65, max_age=3.0,
                        ground_z=0.0, max_range=3.0, confirm_count=4,
                        confirm_radius=.20, confirm_window=6.0,
                        calibration_verified=False, map_frame='map')
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p = lambda k: self.get_parameter(k).value
        self.info = None
        self.buffer = Buffer(cache_time=Duration(seconds=20.))
        self.listener = TransformListener(self.buffer,self)
        self.consensus = Consensus(self.p('confirm_count'), self.p('confirm_radius'), self.p('confirm_window'))
        self.pub = self.create_publisher(PointStamped,'/target/confirmed',10)
        self.health = self.create_publisher(Bool,'/perception/alive',10)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.camera,qos_profile_sensor_data)
        self.create_subscription(DetectionArray,'/yolo/detections',self.detect,qos_profile_sensor_data)

    def camera(self,msg): self.info = msg

    def detect(self,msg):
        info = self.info
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if not self.p('calibration_verified') or self.p('target_class') == 'CHANGE_ME': return
        if info is None or not (0 <= age <= self.p('max_age')): return
        if info.k[0] <= 0 or info.k[4] <= 0 or info.distortion_model not in ('plumb_bob','rational_polynomial'): return
        # DetectionArray's source image frame must match CameraInfo optical frame.
        if msg.header.frame_id != info.header.frame_id or not info.header.frame_id: return
        try:
            tf = self.buffer.lookup_transform(self.p('map_frame'), info.header.frame_id,
                    Time.from_msg(msg.header.stamp),timeout=Duration(seconds=.05)).transform
        except TransformException: return
        q,t = tf.rotation,tf.translation
        R = rotation([q.x,q.y,q.z,q.w]); origin = [t.x,t.y,t.z]
        self.health.publish(Bool(data=True))
        matches = [d for d in msg.detections if d.class_name == self.p('target_class')
                   and d.score >= self.p('confidence')]
        # Ambiguous same-class targets are not silently combined.
        if len(matches) != 1: return
        d = matches[0]; b=d.bbox
        u=b.center.position.x; v=b.center.position.y+b.size.y/2
        if b.size.x <= 0 or b.size.y <= 0: return
        if not (3 < u < info.width-3 and 3 < v < info.height-3): return
        K = np.array(info.k).reshape(3,3)
        D = np.array(info.d,dtype=float)
        xy=cv2.undistortPoints(np.array([[[u,v]]],dtype=np.float64),K,D).reshape(2)
        point=ground_hit([xy[0],xy[1],1.],origin,R,self.p('ground_z'),self.p('max_range'))
        if point is None: return
        stamp=Time.from_msg(msg.header.stamp).nanoseconds/1e9
        result=self.consensus.add(stamp,point)
        if result is None: return
        out=PointStamped(); out.header=msg.header; out.header.frame_id=self.p('map_frame')
        out.point.x,out.point.y,out.point.z=map(float,result)
        self.pub.publish(out)


def main():
    rclpy.init(); n=TargetLocalizer()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()
