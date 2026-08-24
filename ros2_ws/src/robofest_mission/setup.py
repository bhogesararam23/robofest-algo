from setuptools import setup
p='robofest_mission'
setup(name=p,version='0.1.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml']),('share/'+p+'/config',['config/mission.yaml']),('share/'+p+'/launch',['launch/mission.launch.py'])],install_requires=['setuptools'],entry_points={'console_scripts':['mission_state_machine=robofest_mission.mission_state_machine:main','swarm_node=robofest_mission.swarm_node:main','safety_geofence_node=robofest_mission.safety_geofence_node:main']})
