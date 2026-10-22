# TASK 07: Target coordinate visualization; TASK 10: RViz integration.
"""Latched RViz target sphere and coordinate label (search poses use PoseArray)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray


class Visualizer(Node):
    def __init__(self):
        super().__init__('mission_visualizer')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(MarkerArray,'/mission/markers',qos)
        # Clear old markers when this process restarts.
        clear=Marker(); clear.action=Marker.DELETEALL
        self.pub.publish(MarkerArray(markers=[clear]))
        self.create_subscription(PointStamped,'/target/map_position',self.target,qos)

    def target(self,msg):
        sphere=Marker(); sphere.header=msg.header; sphere.ns='target'; sphere.id=0
        sphere.type=Marker.SPHERE; sphere.action=Marker.ADD; sphere.frame_locked=False
        sphere.pose.position=msg.point; sphere.pose.orientation.w=1.0
        sphere.scale.x=sphere.scale.y=sphere.scale.z=.15
        sphere.color.r=1.; sphere.color.a=1.
        label=Marker(); label.header=msg.header; label.ns='target'; label.id=1
        label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD
        label.pose.position.x=msg.point.x; label.pose.position.y=msg.point.y
        label.pose.position.z=msg.point.z+.25; label.pose.orientation.w=1.
        label.scale.z=.12; label.color.r=label.color.g=label.color.b=label.color.a=1.
        label.text=f'Target ({msg.point.x:.2f}, {msg.point.y:.2f}) m'
        self.pub.publish(MarkerArray(markers=[sphere,label]))


def main():
    rclpy.init(); n=Visualizer()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()
