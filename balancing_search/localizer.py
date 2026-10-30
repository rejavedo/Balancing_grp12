# TASK 06: In-node YOLO detection; TASK 07: map-frame target localization.
"""Run YOLO on /image_raw, project the box floor-contact point into the map frame."""
import threading
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
from .core import rotation, ground_hit, Consensus, OPTICAL_TO_LINK, compose


def load_intrinsics(path):
    """K (3x3) and D from a ROS camera_info YAML (camera_calibration output)."""
    with open(path) as f: c = yaml.safe_load(f)
    K = np.array(c['camera_matrix']['data'], dtype=float).reshape(3, 3)
    D = np.array(c['distortion_coefficients']['data'], dtype=float)
    return K, D, int(c['image_width']), int(c['image_height'])


def arr(tf):
    q,t = tf.rotation,tf.translation
    return rotation([q.x,q.y,q.z,q.w]), [t.x,t.y,t.z]


class TargetLocalizer(Node):
    def __init__(self):
        super().__init__('target_localizer')
        defaults = dict(target_class='bottle', confidence=.45, model='/home/ros2_ws/weights/yolo26n.pt',
                        imgsz=640, rate=3.0, image_topic='/image_raw',
                        camera_info_file='/home/ros2_ws/src/balance_usb_cam/config/camera_info.yaml',
                        camera_frame='camera_link', optical_frame=False, odom_frame='odom_combined', max_age=1.0,
                        ground_z=0.0, max_range=3.0, confirm_count=3,
                        confirm_radius=.25, confirm_window=8.0, map_frame='map')
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p = lambda k: self.get_parameter(k).value
        self.K, self.D, self.width, self.height = load_intrinsics(self.p('camera_info_file'))
        from ultralytics import YOLO  # heavy import; only this node needs it
        self.model = YOLO(self.p('model'))
        names = self.model.names
        self.class_id = [i for i,n in names.items() if n == self.p('target_class')]
        if not self.class_id:
            raise RuntimeError(f"target_class '{self.p('target_class')}' not in model classes")
        self.buffer = Buffer(cache_time=Duration(seconds=20.))
        self.listener = TransformListener(self.buffer,self)
        self.consensus = Consensus(self.p('confirm_count'), self.p('confirm_radius'), self.p('confirm_window'))
        self.pub = self.create_publisher(PointStamped,'/target/confirmed',10)
        self.raw = self.create_publisher(PointStamped,'/target/observation',10)
        self.health = self.create_publisher(Bool,'/perception/alive',10)
        self.diag = self.create_publisher(String,'/target/diagnostics',10)
        self.latest = None; self.lock = threading.Lock(); self.busy = False
        self.create_subscription(Image,self.p('image_topic'),self.on_image,qos_profile_sensor_data)
        self.create_timer(1./self.p('rate'),self.tick)
        self.get_logger().info(f"Detecting '{self.p('target_class')}' with {self.p('model')}")

    def on_image(self,msg):
        with self.lock: self.latest = msg

    def tick(self):
        # Single-threaded executor: one frame at a time, newest frame wins.
        with self.lock: msg, self.latest = self.latest, None
        if msg is None: return
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if not -0.5 <= age <= self.p('max_age'): self.say(f'stale image {age:.2f}s'); return
        img = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.width,-1)
        if msg.encoding == 'rgb8': img = img[:,:,::-1]
        elif msg.encoding != 'bgr8': self.say(f'unsupported encoding {msg.encoding}'); return
        res = self.model(np.ascontiguousarray(img), classes=self.class_id, conf=self.p('confidence'),
                         imgsz=self.p('imgsz'), verbose=False)[0]
        self.health.publish(Bool(data=True))
        boxes = [(float(b.conf[0]), b.xyxy[0].tolist()) for b in res.boxes]
        if not boxes: self.say('no target'); return
        conf,(x1,y1,x2,y2) = max(boxes)  # most confident box of the target class
        u,v = (x1+x2)/2, y2
        if not (3 < v < msg.height-3) or x2-x1 <= 0: self.say(f'box clipped conf={conf:.2f}'); return
        sx,sy = self.width/msg.width, self.height/msg.height  # calibration may differ in size
        xy = cv2.undistortPoints(np.array([[[u*sx,v*sy]]]),self.K,self.D).reshape(2)
        ray = np.array([xy[0],xy[1],1.])
        if not self.p('optical_frame'): ray = OPTICAL_TO_LINK @ ray
        # odom->camera at the image time; map->odom is the latest AMCL correction (updates are sparse).
        try:
            oc = self.buffer.lookup_transform(self.p('odom_frame'),self.p('camera_frame'),
                    Time.from_msg(msg.header.stamp),timeout=Duration(seconds=.1)).transform
            mo = self.buffer.lookup_transform(self.p('map_frame'),self.p('odom_frame'),Time()).transform
        except TransformException as e: self.say(f'no TF: {e}'); return
        R,t = compose(*arr(mo),*arr(oc))
        point = ground_hit(ray,t,R,self.p('ground_z'),self.p('max_range'))
        if point is None: self.say(f'ray misses floor conf={conf:.2f}'); return
        obs = PointStamped(); obs.header = msg.header; obs.header.frame_id = self.p('map_frame')
        obs.point.x,obs.point.y,obs.point.z = map(float,point); self.raw.publish(obs)
        self.say(f'seen conf={conf:.2f} at ({point[0]:.2f},{point[1]:.2f})')
        result = self.consensus.add(Time.from_msg(msg.header.stamp).nanoseconds/1e9,point)
        if result is None: return
        out = PointStamped(); out.header = obs.header
        out.point.x,out.point.y,out.point.z = map(float,result)
        self.pub.publish(out)

    def say(self,text): self.diag.publish(String(data=text))


def main():
    rclpy.init(); n=TargetLocalizer()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()
