# TASK 10: ROS package installation and node executable registration.
from setuptools import setup
from glob import glob
setup(name='balancing_search', version='0.2.0', packages=['balancing_search'],
      data_files=[('share/ament_index/resource_index/packages',['resource/balancing_search']),
                  ('share/balancing_search',['package.xml']),
                  ('share/balancing_search/launch',glob('launch/*.launch.py')),
                  ('share/balancing_search/config',glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Project group',maintainer_email='student@example.com',
      description='Map, search, localize a floor-standing target and return home',
      license='MIT',entry_points={'console_scripts':[
          'mission = balancing_search.mission:main',
          'visualizer = balancing_search.visualizer:main',
          'scan_audit = balancing_search.scan_audit:main',
          'target_localizer = balancing_search.localizer:main',
          'velocity_guard = balancing_search.guard:main']})
