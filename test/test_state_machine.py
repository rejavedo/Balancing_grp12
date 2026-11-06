# TASK 11: Mocked mission-state validation; not a physical robot test.
"""Unit-test callbacks with ROS interfaces stubbed; not a ROS integration test."""
import importlib
import sys
import time
import types
import math
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS


def stub(module, **members):
    m=types.ModuleType(module)
    for k,v in members.items(): setattr(m,k,v)
    sys.modules[module]=m


class Message:
    def __init__(self,**kw): self.__dict__.update(kw)


stub('rclpy'); stub('rclpy.node',Node=object); stub('rclpy.action',ActionClient=object)
stub('rclpy.time',Time=object)
stub('rclpy.qos',QoSProfile=object,DurabilityPolicy=object,ReliabilityPolicy=object,qos_profile_sensor_data=object())
stub('action_msgs'); stub('action_msgs.msg',GoalStatus=NS(STATUS_SUCCEEDED=4))
stub('nav2_msgs'); stub('nav2_msgs.action',NavigateToPose=object)
stub('nav_msgs'); stub('nav_msgs.msg',OccupancyGrid=object)
stub('geometry_msgs'); stub('geometry_msgs.msg',PointStamped=Message,PoseStamped=Message,PoseWithCovarianceStamped=Message,PoseArray=Message,Pose=Message,Twist=Message)
stub('std_msgs'); stub('std_msgs.msg',Bool=Message,String=Message)
stub('std_srvs'); stub('std_srvs.srv',Trigger=object,Empty=object)
stub('tf2_ros',Buffer=object,TransformListener=object,TransformException=Exception)
Mission=importlib.import_module('balancing_search.mission').Mission


class Publisher:
    def __init__(self): self.values=[]
    def publish(self,m): self.values.append(m)


def node():
    n=Mission.__new__(Mission)
    n.state='NAVIGATING'; n.heartbeat=Publisher(); n.goal_role='search'
    n.perception=time.monotonic(); n.sent=time.monotonic(); n.started=time.monotonic()
    n.p=lambda key:{'perception_timeout':4.,'goal_timeout':90.,'mission_timeout':1200.,'home_tolerance':.2}[key]
    n.pose=lambda:(1.,2.,0.); n.localized=lambda:True
    n.result_future=Future(); n.send_future=None; n.target=None
    n.transition=lambda state,detail='': setattr(n,'state',state)
    n.home=(1.,2.,0.); n.return_tries=2
    n.goal_handle=NS(cancel_goal_async=lambda:Future())
    n.index=0; n.failures=0
    n.cmd=Publisher(); n.stop_until=-float('inf'); n.created=time.monotonic(); n.next_try=0.
    return n


class StateTests(unittest.TestCase):
    def test_target_waits_for_cancel_before_return(self):
        n=node(); n.target=object(); returned=[]
        n.return_home=lambda:returned.append(True)
        n.tick()
        self.assertEqual(n.state,'CANCELLING')
        self.assertFalse(n.heartbeat.values[-1].data)
        self.assertEqual(returned,[])
        n.tick(); self.assertEqual(returned,[])
        n.result_future.set_result(NS(status=5))
        n.tick(); self.assertEqual(returned,[True])

    def test_abort_cancels_late_goal_acceptance(self):
        n=node(); n.state='FAULT'; calls=[]
        n.send_future=Future()
        n.send_future.set_result(NS(accepted=True,cancel_goal_async=lambda:calls.append('cancel')))
        n.tick()
        self.assertEqual(calls,['cancel']); self.assertFalse(n.heartbeat.values[-1].data)

    def test_return_failure_never_success(self):
        n=node(); n.goal_role='return'; n.result_future.set_result(NS(status=6))
        n.finish=lambda:self.fail('False success')
        n.complete_goal(); self.assertEqual(n.state,'FAULT')

    def test_success_status_alone_not_enough(self):
        n=node(); n.goal_role='return'; n.result_future.set_result(NS(status=4))
        n.pose=lambda:(3.,4.,0.)
        n.finish=lambda:self.fail('Outside home tolerance')
        n.complete_goal(); self.assertEqual(n.state,'FAULT')

    def test_verified_home_arrival(self):
        n=node(); n.goal_role='return'; n.result_future.set_result(NS(status=4)); done=[]
        n.finish=lambda:done.append(True)
        n.complete_goal(); self.assertEqual(done,[True])

    def test_stale_perception_fault(self):
        n=node(); n.perception=0.; n.tick()
        self.assertEqual(n.state,'FAULT'); self.assertFalse(n.heartbeat.values[-1].data)

    def test_localization_loss_fault(self):
        n=node(); n.pose=lambda:None; n.tick()
        self.assertEqual(n.state,'FAULT'); self.assertFalse(n.heartbeat.values[-1].data)

    def test_cancel_timeout_disables_motion(self):
        n=node(); n.state='CANCELLING'; n.cancel_time=time.monotonic()-6
        n.tick(); self.assertEqual(n.state,'FAULT')
        self.assertFalse(n.heartbeat.values[-1].data)

    def test_amcl_start_covariance_vs_report_gate(self):
        module=sys.modules['balancing_search.mission']
        old=module.Time
        class Stamp:
            def __init__(self,n): self.nanoseconds=n
            def __sub__(self,other): return Stamp(self.nanoseconds-other.nanoseconds)
        module.Time=NS(from_msg=lambda stamp:Stamp(stamp))
        try:
            n=node(); n.localized=Mission.localized.__get__(n)
            values={'amcl_timeout':15.,'max_xy_variance':.3,'max_yaw_variance':.15,
                    'report_xy_variance':.1,'report_yaw_variance':.1}
            n.p=lambda k:values[k]
            n.get_clock=lambda:NS(now=lambda:Stamp(100*10**9))
            cov=[0.]*36; cov[0]=cov[7]=.25; cov[35]=.07
            n.amcl=NS(header=NS(stamp=100*10**9),pose=NS(covariance=cov))
            n.amcl_time=time.monotonic()
            self.assertTrue(n.localized())
            self.assertFalse(n.localized(for_report=True))
            cov[0]=cov[7]=.02
            self.assertTrue(n.localized(for_report=True))
            n.amcl.header.stamp=80*10**9
            self.assertFalse(n.localized()) # Latched old sample is still stale.
        finally: module.Time=old

    def test_target_report_rejects_old_future_and_wrong_frame(self):
        module=sys.modules['balancing_search.mission']
        old=module.Time
        class Stamp:
            def __init__(self,n): self.nanoseconds=n
            def __sub__(self,other): return Stamp(self.nanoseconds-other.nanoseconds)
        module.Time=NS(from_msg=lambda stamp:Stamp(stamp))
        try:
            n=node(); n.target_after=90*10**9
            n.get_clock=lambda:NS(now=lambda:Stamp(100*10**9))
            n.localized=lambda for_report=False:True
            n.get_logger=lambda:NS(info=lambda text:None)
            n.report=Publisher(); saved=[]; n.save_report=lambda outcome:saved.append(outcome)
            for stamp,frame in [(89,'map'),(95,'map'),(101,'map'),(100,'odom')]:
                n.on_target(NS(header=NS(stamp=stamp*10**9,frame_id=frame),point=NS(x=1.,y=2.,z=0.)))
                self.assertIsNone(n.target)
            n.on_target(NS(header=NS(stamp=99*10**9,frame_id='map'),point=NS(x=1.,y=2.,z=0.)))
            self.assertIsNotNone(n.target)
            self.assertEqual(len(n.report.values),1)
            self.assertEqual(saved,['TARGET_REPORTED'])
        finally: module.Time=old

    def test_start_needs_map(self):
        n=node(); n.state='IDLE'; n.map=None; n.client=NS(server_is_ready=lambda:True)
        response=NS(success=False,message='')
        n.start(None,response)
        self.assertFalse(response.success)
        self.assertIn('Need map',response.message)

    def test_fault_holds_robot_still(self):
        n=node(); n.pose=lambda:None; n.tick()
        self.assertEqual(n.state,'FAULT')
        n.tick(); self.assertEqual(len(n.cmd.values),1)
        self.assertFalse(hasattr(n.cmd.values[0],'linear'))  # stub Twist() == zero command

    def test_failed_goal_skips_rest_of_viewpoint(self):
        n=node(); n.goals=[(0,0,0,0),(0,0,1,0),(0,0,2,0),(1,1,0,1)]; n.index=1
        n.result_future.set_result(NS(status=6)); n.complete_goal()
        self.assertEqual((n.state,n.index,n.failures),('SEARCH',3,1))

    def test_view_goals_face_arrival_direction(self):
        g=sys.modules['balancing_search.mission'].view_goals([(0.,0.),(0.,2.)],(0.,0.,0.3),6)
        self.assertEqual(len(g),12); self.assertAlmostEqual(g[0][2],0.3)
        self.assertAlmostEqual(g[6][2],math.pi/2); self.assertEqual(g[6][3],1)


if __name__=='__main__': unittest.main(verbosity=2)
