from setuptools import setup
p='robofest_drivers'
setup(name=p,version='0.1.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])],install_requires=['setuptools'],entry_points={'console_scripts':['fc_bridge_node=robofest_drivers.fc_bridge_node:main']})
