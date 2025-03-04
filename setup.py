from setuptools import find_packages, setup
import os
from glob import glob

package_name='mobrob'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'sensors_raw=mobrob.sensors_raw:main',
		'sensors_processor=mobrob.sensors_processor:main',
		'wheel_control=mobrob.wheel_control:main',
		'set_desired_wheel_speeds=mobrob.set_desired_wheel_speeds:main', 
		'mobile_robot_kinematic_simulator=mobrob.mobile_robot_kinematic_simulator:main', 
		'mobile_robot_animator=mobrob.mobile_robot_animator:main', 
	    'set_desired_wheel_speeds_by_path_specs=mobrob.set_desired_wheel_speeds_by_path_specs:main',
        'dead_reckoning=mobrob.dead_reckoning:main',
        ## Add an "entry point" to the file "closed_loop_path_follower.py", function "main", and "set_path_to_follow.py", function "main". They will be much like the others above. 
        ## Remember a comma after the previous one: we are continuing an existing list. 
        ## Then save this file, Removing "_HWK" removed from the name.
        'closed_loop_path_follower=mobrob.closed_loop_path_follower:main',
        'set_path_to_follow=mobrob.set_path_to_follow:main'
        ],
    },
)
