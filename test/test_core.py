# TASK 11: Offline geometry, coverage and control validation.
import math
import numpy as np
import unittest
from balancing_search.core import coverage, rotation, ground_hit, Consensus, limited_twist, OPTICAL_TO_LINK


def test_optical_to_ground():
    # Optical z forward, x right, y down; base x forward/y left/z up.
    R=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    assert np.allclose(ground_hit([0,.5,1],[0,0,.5],R),[1,0,0])
    assert np.allclose(ground_hit([.25,.5,1],[0,0,.5],R),[1,-.25,0])


def test_horizon_behind_and_far_rejected():
    R=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    assert ground_hit([0,0,1],[0,0,.5],R) is None
    assert ground_hit([0,-1,1],[0,0,.5],R) is None
    assert ground_hit([0,.15,1],[0,0,.5],R,max_range=2) is None


def test_map_frame_translation_rotation():
    R=rotation([0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
    assert np.allclose(R @ [1,0,0],[0,1,0],atol=1e-12)
    optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    assert np.allclose(ground_hit([0,.5,1],[2,3,.5],R@optical),[2,4,0])


def test_bad_quaternion():
    with unittest.TestCase().assertRaises(ValueError): rotation([0,0,0,0])


def test_temporal_consensus_duplicates_outliers():
    c=Consensus(count=3)
    assert c.add(1,[1,2,0]) is None
    assert c.add(1,[1,2,0]) is None
    assert c.add(2,[1.01,2,0]) is None
    assert np.allclose(c.add(3,[.99,2,0]),[1,2,0])
    assert c.add(4,[4,4,0]) is None
    assert c.add(12,[4,4,0]) is None


def test_unknown_wall_disconnected_map():
    a=np.zeros((30,30),dtype=int); a[:,15]=-1
    pts=coverage(a,.1,(0,0,0),(.7,1.),.2,.4)
    assert pts and all(x < 1.5 for x,y in pts)
    assert all(x > .2 and y > .2 and y < 2.8 for x,y in pts)


def test_rotated_origin():
    a=np.zeros((20,20),dtype=int)
    pts=coverage(a,.1,(4,5,math.pi/2),(3,6),.2,.4)
    assert pts and all(2 < x < 4 and 5 < y < 7 for x,y in pts)


def test_invalid_start():
    with unittest.TestCase().assertRaises(ValueError): coverage(np.zeros((10,10)),.1,(0,0,0),(0,0),.2,.4)


def test_no_diagonal_corner_cut():
    a=np.full((9,9),100); a[1:4,1:4]=0; a[4:7,4:7]=0
    pts=coverage(a,.1,(0,0,0),(.25,.25),.001,.1)
    assert all(x < .4 and y < .4 for x,y in pts)


def test_velocity_limits_and_nan():
    assert np.allclose(limited_twist(10,10,(0,0),.1),(.015,.04))
    assert np.allclose(limited_twist(-1,-10,(0,0),.1),(0,-.04))
    assert limited_twist(float('nan'),0,(.1,.1),.1)==(0,0)

def test_near_wall_uses_reachable_viewpoint():
    a=np.zeros((60,60))
    # Cell centre 0.425m from outer border: valid for transit, not viewpoint margin.
    pts=coverage(a,.05,(0,0,0),(.425,1.5),.50,1.2,transit_clearance=.38)
    assert pts and pts[0][0] > .5


def test_seed_never_crosses_wall():
    a=np.zeros((40,40)); a[:,9]=100
    # Narrow left chamber fits transit but has no safe viewpoint. The nearest
    # geometric safe cell across the wall must NOT be selected.
    with unittest.TestCase().assertRaises(ValueError):
        coverage(a,.05,(0,0,0),(.225,1.),.3,.6,transit_clearance=.1)


def test_seed_rejects_physically_unsafe_start():
    with unittest.TestCase().assertRaises(ValueError):
        coverage(np.zeros((60,60)),.05,(0,0,0),(.10,1.),.42,1.2,transit_clearance=.38)


def test_spacing_reduces_goal_count():
    a=np.zeros((80,80))
    old=coverage(a,.05,(0,0,0),(2.,2.),.42,.6)
    new=coverage(a,.05,(0,0,0),(2.,2.),.42,1.2)
    assert len(new) < len(old)


def test_heading_fov_overlap():
    from balancing_search.core import heading_count
    fx=640/(2*math.tan(math.radians(60)/2))
    assert heading_count(6,640,fx)==8
    assert heading_count(6,640,320)==6


def test_scan_zero_strict_vs_bridged():
    from balancing_search.core import checked_scan
    a=np.full(360,2.); a[10]=0.
    assert checked_scan(a,.05,8.,math.pi/180)[0] is None
    out,_=checked_scan(a,.05,8.,math.pi/180,.01,2.)
    assert out is not None and out[10]==2.


def test_scan_preserves_near_obstacle():
    from balancing_search.core import checked_scan
    a=np.full(360,2.); a[10]=0.; a[9]=.2
    out,_=checked_scan(a,.05,8.,math.pi/180,.01,2.)
    assert out is not None and out[10]==.2 and min(out)==.2


def test_scan_wrap_gap_and_large_gap_rejected():
    from balancing_search.core import checked_scan,scan_quality
    a=np.full(360,2.); a[:3]=0.; a[-3:]=0.
    assert math.isclose(scan_quality(a,.05,8.,math.pi/180)['longest_invalid_gap_deg'],6.)
    assert checked_scan(a,.05,8.,math.pi/180,.1,2.)[0] is None


def test_scan_missing_brackets_rejected():
    from balancing_search.core import checked_scan
    a=np.full(360,2.); a[10]=float('nan'); a[9]=float('inf')
    assert checked_scan(a,.05,8.,math.pi/180,.01,2.)[0] is None


def test_scan_all_invalid_rejected():
    from balancing_search.core import checked_scan
    for value in (0.,float('nan'),float('inf')):
        assert checked_scan([value]*360,.05,8.,math.pi/180,.1,2.)[0] is None


def test_camera_link_forward_ray_hits_floor_ahead():
    # Pixel below the principal point, camera_link 0.27 m up, robot at map (1,2) facing +y.
    ray=OPTICAL_TO_LINK@np.array([0.,0.5,1.])
    yaw=np.pi/2; q=[0,0,np.sin(yaw/2),np.cos(yaw/2)]
    p=ground_hit(ray,[1.,2.,.27],rotation(q))
    assert abs(p[0]-1.)<1e-9 and abs(p[1]-(2+.54))<1e-9 and abs(p[2])<1e-9


def test_compose_map_odom_base():
    from balancing_search.core import compose
    yaw=np.pi/2; Rz=rotation([0,0,np.sin(yaw/2),np.cos(yaw/2)])
    R,t=compose(Rz,[1.,0.,0.],np.eye(3),[2.,0.,0.])  # base 2 m ahead in odom, odom rotated 90 deg in map
    assert np.allclose(t,[1.,2.,0.]) and np.allclose(R,Rz)


if __name__ == "__main__":
    suite=unittest.TestSuite(unittest.FunctionTestCase(v) for k,v in list(globals().items()) if k.startswith("test_"))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
