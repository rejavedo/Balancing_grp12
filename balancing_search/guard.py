# TASK 01: Robot interface and safe velocity output; TASK 10: ROS topics/services.
"""Run ON ROBOT. Sole /cmd_vel publisher during autonomous operation."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from .core import limited_twist, checked_scan


class Guard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        for k,v in dict(hardware_verified=False,cmd_timeout=.4,heartbeat_timeout=.6,
                        scan_timeout=.5,stop_range=.38,max_v=.15,max_w=.30,
                        acceleration=.15,angular_acceleration=.4,
                        max_invalid_fraction=0.,max_invalid_gap_deg=0.).items():
            self.declare_parameter(k,v)
        self.p=lambda k:self.get_parameter(k).value
        self.cmd=Twist(); self.tc=self.th=self.ts=-math.inf
        self.enabled=False; self.estop=False; self.clear=False
        self.reason='No scan received'; self.last_status=None
        self.ready_pub=self.create_publisher(Bool,'/guard/ready',10)
        self.status_pub=self.create_publisher(String,'/guard/status',10)
        self.previous=(0.,0.); self.last=time.monotonic()
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(Twist,'/nav_cmd_vel',self.command,10)
        self.create_subscription(Bool,'/mission/heartbeat',self.heartbeat,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_service(Trigger,'/mission/estop',self.stop)
        self.create_service(Trigger,'/mission/reset_estop',self.reset)
        self.create_timer(.05,self.tick)

    def command(self,msg): self.cmd=msg; self.tc=time.monotonic()
    def heartbeat(self,msg):
        self.enabled=msg.data; self.th=time.monotonic()
        if not msg.data: self.cmd=Twist(); self.tc=-math.inf
    def stop(self,req,res):
        self.estop=True; res.success=True; res.message='Latched stop'; return res
    def reset(self,req,res):
        if self.enabled:
            res.success=False; res.message='Abort mission before reset'
        else:
            self.estop=False; res.success=True; res.message='Reset; mission remains disabled'
        return res
    def scan(self,msg):
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        ranges,reason=checked_scan(msg.ranges,msg.range_min,msg.range_max,msg.angle_increment,
                                 self.p('max_invalid_fraction'),self.p('max_invalid_gap_deg'))
        self.clear=(0 <= age < self.p('scan_timeout') and ranges is not None
                    and min(ranges) > self.p('stop_range'))
        if not 0 <= age < self.p('scan_timeout'): reason='stale/future scan stamp'
        elif ranges is not None and not self.clear: reason='obstacle inside stop_range'
        self.reason=reason
        self.ts=time.monotonic()
    def tick(self):
        now=time.monotonic(); dt=min(.1,now-self.last); self.last=now
        ready=(self.p('hardware_verified') and not self.estop and self.clear
               and now-self.ts < self.p('scan_timeout'))
        self.ready_pub.publish(Bool(data=ready))
        reason=('hardware not verified' if not self.p('hardware_verified') else
                'estop latched' if self.estop else
                'scan stream stale' if now-self.ts >= self.p('scan_timeout') else self.reason)
        self.status_pub.publish(String(data=reason))
        if reason != self.last_status:
            self.get_logger().info('Guard: '+reason); self.last_status=reason
        safe=(self.p('hardware_verified') and self.enabled and not self.estop and self.clear
              and now-self.tc < self.p('cmd_timeout') and now-self.th < self.p('heartbeat_timeout')
              and now-self.ts < self.p('scan_timeout'))
        if safe:
            self.previous=limited_twist(self.cmd.linear.x,self.cmd.angular.z,self.previous,dt,
                self.p('max_v'),self.p('max_w'),self.p('acceleration'),self.p('angular_acceleration'))
        else: self.previous=(0.,0.)
        out=Twist(); out.linear.x,out.angular.z=self.previous; self.pub.publish(out)


def main():
    rclpy.init(); n=Guard()
    try: rclpy.spin(n)
    finally:
        n.pub.publish(Twist()); n.destroy_node(); rclpy.shutdown()
