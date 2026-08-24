from setuptools import setup
package_name = 'robofest_navigation'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
 ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
 ('share/' + package_name, ['package.xml']),
 ('share/' + package_name + '/config', ['config/ekf.yaml']),
 ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
], install_requires=['setuptools'], zip_safe=True, entry_points={'console_scripts': [
 'mine_mapping_node = robofest_navigation.mine_mapping_node:main',
 'path_planner_node = robofest_navigation.path_planner_node:main',
]})
