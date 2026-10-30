# TASKS 03, 04, 05, 07, 08, 09, 10: integrated autonomous mission; see labelled methods.
"""Nonblocking Nav2 mission: start -> coverage -> report -> return -> stop."""
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from types import SimpleNamespace as NS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, Empty
from tf2_ros import Buffer, TransformListener, TransformException
from .core import coverage, heading_count, compose, rotation


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Mission(Node):
    def __init__(self):
        super().__init__('search_mission')
        for k,v in dict(base_frame='base_footprint', odom_frame='odom_combined', clearance=.30, transit_clearance=.20, max_seed_distance=1.,
                        spacing=0.9, headings=6, heading_overlap=.15, hfov_deg=93.0,
                        auto_start=True, auto_start_delay=10., set_initial_pose=True,
                        initial_x=0., initial_y=0., initial_yaw=0.,
                        dwell=2.5, goal_timeout=60., mission_timeout=900.,
                        home_tolerance=.20, max_xy_variance=.30, max_yaw_variance=.15,
                        report_xy_variance=.10, report_yaw_variance=.10,
                        amcl_timeout=15., nomotion_interval=5., perception_timeout=4.,
                        report_path='~/balancing_mission_result.json').items():
            self.declare_parameter(k,v)
        self.p=lambda k:self.get_parameter(k).value
        self.state='IDLE'; self.map=None; self.amcl=None; self.amcl_time=-math.inf
        self.created=time.monotonic(); self.next_try=0.; self.stop_until=-math.inf
        self.perception=-math.inf; self.home=None; self.target=None; self.reported=False
        self.goals=[]; self.index=0; self.failures=0; self.return_tries=0
        self.goal_handle=None; self.send_future=None; self.result_future=None; self.cancel_future=None
        self.cancel_reason=None; self.goal_role=None; self.target_after=0
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.client=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.nomotion=self.create_client(Empty,'/request_nomotion_update')
        self.nomotion_future=None; self.nomotion_sent=-math.inf
        self.create_timer(1.,self.refresh_amcl)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.status=self.create_publisher(String,'/mission/status',latched)
        self.views=self.create_publisher(PoseArray,'/search/viewpoints',latched)
        self.report=self.create_publisher(PointStamped,'/target/map_position',latched)
        self.heartbeat=self.create_publisher(Bool,'/mission/heartbeat',10)
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10); self.init_sent=0
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,latched)
        self.create_subscription(Bool,'/perception/alive',self.on_perception,10)
        self.create_subscription(PointStamped,'/target/confirmed',self.on_target,10)
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/abort',self.abort)
        self.create_timer(.1,self.tick)
        self.transition('IDLE')

    def transition(self,state,detail=''):
        self.state=state; text=state+(': '+detail if detail else '')
        self.status.publish(String(data=text)); self.get_logger().info(text)
    # TASK 03: Request fresh localization while stationary.
    def refresh_amcl(self):
        # Bounded, asynchronous scan updates while stationary; no commanded spin.
        if self.state in ('FAULT','SUCCESS','TARGET_NOT_FOUND'): return
        now=time.monotonic()
        if now-self.nomotion_sent < self.p('nomotion_interval'): return
        if not self.nomotion.service_is_ready(): return
        if self.nomotion_future is not None and not self.nomotion_future.done():
            self.nomotion.remove_pending_request(self.nomotion_future)
        self.nomotion_future=self.nomotion.call_async(Empty.Request())
        self.nomotion_sent=now
    def on_map(self,msg): self.map=msg
    def on_amcl(self,msg): self.amcl=msg; self.amcl_time=time.monotonic()
    def on_perception(self,msg):
        if msg.data: self.perception=time.monotonic()
    # TASK 03: Tell AMCL where the assigned start is (map origin = mapping start by default).
    def send_initial_pose(self):
        m=PoseWithCovarianceStamped(); m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg()
        m.pose.pose.position.x=float(self.p('initial_x')); m.pose.pose.position.y=float(self.p('initial_y'))
        th=float(self.p('initial_yaw')); m.pose.pose.orientation.z=math.sin(th/2); m.pose.pose.orientation.w=math.cos(th/2)
        cov=[0.]*36; cov[0]=cov[7]=.05**2; cov[35]=.1**2; m.pose.covariance=cov
        self.initial.publish(m); self.init_sent+=1
        self.get_logger().info(f'Initial pose sent to AMCL: ({m.pose.pose.position.x:.2f}, {m.pose.pose.position.y:.2f}, {th:.2f} rad)')

    # TASK 03: Read fresh map-frame robot pose.
    def pose(self):
        # AMCL republishes map->odom only when it updates (~5 s apart while still), so
        # compose the latest map->odom with the fresh odom->base instead of one chained lookup.
        try:
            mo=self.buffer.lookup_transform('map',self.p('odom_frame'),Time())
            ob=self.buffer.lookup_transform(self.p('odom_frame'),self.p('base_frame'),Time())
        except TransformException: return None
        now=self.get_clock().now()
        age=lambda tf:(now-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        stamp=Time.from_msg(ob.header.stamp).nanoseconds
        if age(mo) > self.p('amcl_timeout') or (stamp != 0 and not age(ob) < 1.): return None
        q=lambda tf:[tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]
        v=lambda tf:[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
        R,t=compose(rotation(q(mo)),v(mo),rotation(q(ob)),v(ob))
        return (float(t[0]),float(t[1]),math.atan2(R[1,0],R[0,0]))
    # TASK 03: Check localization age and covariance.
    def localized(self,for_report=False):
        if self.amcl is None or time.monotonic()-self.amcl_time > self.p('amcl_timeout'): return False
        age=(self.get_clock().now()-Time.from_msg(self.amcl.header.stamp)).nanoseconds/1e9
        if not 0 <= age <= self.p('amcl_timeout'): return False
        cov=self.amcl.pose.covariance
        xy=self.p('report_xy_variance' if for_report else 'max_xy_variance')
        ya=self.p('report_yaw_variance' if for_report else 'max_yaw_variance')
        return all(math.isfinite(cov[i]) and 0 <= cov[i] <= limit
                   for i,limit in ((0,xy),(7,xy),(35,ya)))
    # TASKS 05, 08, 09: Check readiness, generate search goals and capture home.
    def start(self,req,res):
        if self.state != 'IDLE':
            res.message='Restart mission node for a new mission'; return res
        home=self.pose()
        missing=[k for k,bad in (('map',self.map is None),('map->base TF',home is None),
                                 ('AMCL pose/covariance',not self.localized()),
                                 ('perception',time.monotonic()-self.perception > self.p('perception_timeout')),
                                 ('Nav2 navigate_to_pose',not self.client.server_is_ready())) if bad]
        if missing:
            res.message='Need map, fresh TF/AMCL, live perception, active Nav2; missing: '+', '.join(missing); return res
        try:
            m=self.map; o=m.info.origin
            if m.header.frame_id != 'map': raise ValueError('Map frame must be map')
            pts=coverage(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,
                         (o.position.x,o.position.y,yaw(o.orientation)),home[:2],
                         self.p('clearance'),self.p('spacing'),self.p('transit_clearance'),
                         self.p('max_seed_distance'))
            fx=1./(2*math.tan(math.radians(self.p('hfov_deg'))/2))  # unit-width image
            n=heading_count(self.p('headings'),1.,fx,self.p('heading_overlap'))
            self.goals=[(x,y,2*math.pi*k/n) for x,y in pts for k in range(n)]
        except ValueError as e:
            res.message=str(e); return res
        views=PoseArray(); views.header.frame_id='map'; views.header.stamp=self.get_clock().now().to_msg()
        for x,y,theta in self.goals:
            pose=Pose(); pose.position.x=float(x); pose.position.y=float(y)
            pose.orientation.z=math.sin(theta/2); pose.orientation.w=math.cos(theta/2)
            views.poses.append(pose)
        self.views.publish(views)
        dwell_only=len(self.goals)*self.p('dwell')
        self.get_logger().info(f'{len(pts)} viewpoints, {n} headings; dwell alone {dwell_only:.0f}s (travel/turning extra)')
        self.home=home; self.started=time.monotonic()
        self.target_after=self.get_clock().now().nanoseconds
        self.transition('SEARCH'); res.success=True
        res.message=f'Home captured; {len(self.goals)} view poses generated'; return res
    def abort(self,req,res):
        self.fault('Operator abort'); res.success=True; res.message='Motion disabled'; return res
    def fault(self,why):
        self.transition('FAULT',why); self.heartbeat.publish(Bool(data=False))
        self.stop_until=time.monotonic()+2.
        if self.goal_handle is not None: self.goal_handle.cancel_goal_async()
    # TASK 07: Report a confirmed target in map-frame metres.
    def on_target(self,msg):
        if self.state not in ('SEARCH','NAVIGATING','DWELL','SENDING') or self.goal_role == 'return': return
        if msg.header.frame_id != 'map' or Time.from_msg(msg.header.stamp).nanoseconds < self.target_after: return
        if not np.isfinite([msg.point.x,msg.point.y,msg.point.z]).all(): return
        if not self.localized(for_report=True) or self.pose() is None: return
        if self.target is None:
            self.target=msg
            self.get_logger().info(f'TARGET (map, metres): ({msg.point.x:.3f}, {msg.point.y:.3f})')
            self.report.publish(msg)
            try: self.save_report('TARGET_REPORTED')
            except OSError as e:
                self.fault(f'Cannot save target report: {e}'); return
            self.reported=True
    def save_report(self,outcome):
        p=Path(self.p('report_path')).expanduser(); p.parent.mkdir(parents=True,exist_ok=True)
        data=dict(outcome=outcome,frame='map',units='m',home=list(self.home),
                  target=None if self.target is None else dict(x=self.target.point.x,y=self.target.point.y),
                  failed_search_goals=self.failures)
        tmp=p.with_suffix('.tmp'); tmp.write_text(json.dumps(data,indent=2)); os.replace(tmp,p)
    # TASK 04: Send an asynchronous navigation goal.
    def send(self,pose,role):
        goal=NavigateToPose.Goal(); goal.pose=PoseStamped()
        goal.pose.header.frame_id='map'; goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(pose[0]); goal.pose.pose.position.y=float(pose[1])
        goal.pose.pose.orientation.z=math.sin(pose[2]/2); goal.pose.pose.orientation.w=math.cos(pose[2]/2)
        self.goal_role=role; self.sent=time.monotonic()
        self.send_future=self.client.send_goal_async(goal); self.transition('SENDING',role)
    def cancel(self,reason):
        self.cancel_reason=reason; self.cancel_time=time.monotonic()
        self.cancel_future=self.goal_handle.cancel_goal_async(); self.transition('CANCELLING',reason)
    # TASK 08: Navigate back to the captured starting pose.
    def return_home(self):
        self.return_tries+=1; self.send(self.home,'return')
    # TASK 08: Record outcome and disable commanded motion.
    def finish(self):
        outcome='SUCCESS' if self.target is not None else 'TARGET_NOT_FOUND'
        try: self.save_report(outcome)
        except OSError as e: self.fault(str(e)); return
        self.transition(outcome); self.heartbeat.publish(Bool(data=False))
        self.stop_until=time.monotonic()+2.
        t=self.target
        self.get_logger().info('MISSION '+outcome+('' if t is None else
                               f' — TARGET (map, metres): ({t.point.x:.3f}, {t.point.y:.3f})'))
    # TASK 09: Advance the nonblocking mission state machine.
    def tick(self):
        active=self.state not in ('IDLE','FAULT','SUCCESS','TARGET_NOT_FOUND')
        now=time.monotonic()
        # Late goal acceptance after abort must still be cancelled.
        if not active:
            self.heartbeat.publish(Bool(data=False))
            if now < self.stop_until: self.cmd.publish(Twist())  # hold still after finish/fault
            if self.state=='IDLE' and self.p('set_initial_pose') and self.init_sent < 3 and now >= self.next_try \
                    and self.initial.get_subscription_count() > 0:
                self.send_initial_pose(); self.next_try=now+1.; return
            if (self.state=='IDLE' and self.p('auto_start') and now >= self.next_try
                    and now-self.created >= self.p('auto_start_delay')):
                self.next_try=now+2.
                r=self.start(None,NS(success=False,message=''))
                if not r.success: self.get_logger().info('Waiting to start: '+r.message,throttle_duration_sec=10.)
            if self.send_future is not None and self.send_future.done():
                h=self.send_future.result(); self.send_future=None
                if h is not None and h.accepted: h.cancel_goal_async()
            return
        if self.pose() is None or not self.localized(): self.fault('Localization lost'); return
        searching=self.goal_role != 'return'
        if searching and now-self.perception > self.p('perception_timeout'):
            self.fault('Perception stale'); return
        # Do not allow movement during cancellation or after report until return goal starts.
        self.heartbeat.publish(Bool(data=self.state in ('SENDING','NAVIGATING') and
                                    (self.target is None or self.goal_role == 'return')))
        if self.state == 'SEARCH':
            if self.target is not None or self.index >= len(self.goals) or now-self.started > self.p('mission_timeout'):
                self.return_home()
            else: self.send(self.goals[self.index],'search')
        elif self.state == 'SENDING':
            if now-self.sent > 10.: self.fault('Goal response timeout'); return
            if self.send_future.done():
                self.goal_handle=self.send_future.result(); self.send_future=None
                if not self.goal_handle.accepted:
                    self.goal_handle=None
                    if self.goal_role=='return': self.fault('Return goal rejected')
                    else: self.failures+=1; self.index+=1; self.transition('SEARCH')
                else:
                    self.result_future=self.goal_handle.get_result_async(); self.transition('NAVIGATING',self.goal_role)
        elif self.state == 'NAVIGATING':
            if self.result_future.done(): self.complete_goal()
            elif self.target is not None and self.goal_role == 'search': self.cancel('target')
            elif now-self.sent > self.p('goal_timeout'): self.cancel('timeout')
            elif self.goal_role=='search' and now-self.started > self.p('mission_timeout'): self.cancel('budget')
        elif self.state == 'CANCELLING':
            if self.result_future.done():
                self.goal_handle=None
                if self.cancel_reason in ('target','budget'): self.return_home()
                elif self.goal_role=='return':
                    if self.return_tries < 2: self.return_home()
                    else: self.fault('Return timed out')
                else: self.failures+=1; self.index+=1; self.transition('SEARCH')
            elif now-self.cancel_time > 5.: self.fault('Cancellation did not complete')
        elif self.state == 'DWELL':
            if self.target is not None: self.return_home()
            elif now-self.dwell_start >= self.p('dwell'):
                self.index+=1; self.transition('SEARCH')
    # TASKS 04, 08: Handle goal result and verify return tolerance.
    def complete_goal(self):
        result=self.result_future.result(); self.goal_handle=None
        if self.goal_role=='return':
            pose=self.pose()
            if (result.status==GoalStatus.STATUS_SUCCEEDED and pose is not None
                    and math.hypot(pose[0]-self.home[0],pose[1]-self.home[1]) <= self.p('home_tolerance')):
                self.finish()
            elif self.return_tries < 2: self.return_home()
            else: self.fault('Could not verify arrival at home')
        elif self.target is not None: self.return_home()
        elif result.status==GoalStatus.STATUS_SUCCEEDED:
            self.dwell_start=time.monotonic(); self.transition('DWELL')
        else:
            self.failures+=1; self.index+=1; self.transition('SEARCH')


def main():
    rclpy.init(); n=Mission()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.heartbeat.publish(Bool(data=False)); n.destroy_node(); rclpy.shutdown()
