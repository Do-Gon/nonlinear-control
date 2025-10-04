from setuptools import find_packages, setup

package_name = 'jetcobot_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='dk3322@columbia.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tutorial_ctrl = jetcobot_ctrl.send_data_tutorial:main',
            'tutorial_coords = jetcobot_ctrl.send_coords_tutorial:main',
            'read_angles = jetcobot_ctrl.read_angles:main',
            'read_coords = jetcobot_ctrl.read_coords:main',
            'move_object = jetcobot_ctrl.hardcoded_move_object:main',
            'ik_solve = jetcobot_ctrl.inverse_kinematics_solver:main',
            'gripper = jetcobot_ctrl.gripper:main',
        ],
    },
)
