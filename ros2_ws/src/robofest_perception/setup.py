from setuptools import setup
package_name='robofest_perception'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/config',['config/vision.yaml'])], install_requires=['setuptools'], entry_points={'console_scripts':['vision_pipeline_node=robofest_perception.vision_pipeline_node:main','fake_vision_node=robofest_perception.fake_vision_node:main']})
