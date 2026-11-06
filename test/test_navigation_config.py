"""Offline configuration contracts; ROS bringup must also be checked on the Pi."""
import ast
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_velocity_smoother_input_is_not_remapped_to_output():
    tree = ast.parse((ROOT / 'launch/mission.launch.py').read_text())
    remaps = []
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'SetRemap':
            remaps.append({kw.arg: ast.literal_eval(kw.value) for kw in call.keywords})
    assert {'src': 'cmd_vel_nav', 'dst': 'cmd_vel'} not in remaps
    assert not remaps  # rely on the upstream Humble input/output wiring


def test_navigation_endpoint_matches_mission_accuracy():
    nav = yaml.safe_load((ROOT / 'config/nav2.yaml').read_text())
    mission = yaml.safe_load((ROOT / 'config/mission.yaml').read_text())['search_mission']['ros__parameters']
    planner = nav['planner_server']['ros__parameters']['GridBased']
    goal = nav['controller_server']['ros__parameters']['general_goal_checker']
    assert planner['allow_unknown'] is False
    assert planner['use_astar'] is True
    assert planner['tolerance'] <= goal['xy_goal_tolerance'] <= mission['home_tolerance']
