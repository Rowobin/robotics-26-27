from setuptools import find_packages, setup

package_name = 'lite6_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cazevedo',
    maintainer_email='cazevedo@ipn.pt',
    description='Python nodes for the UFACTORY Lite 6 (Robotics 02000537).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # `ros2 run lite6_control <name>`
            'joint_echo = lite6_control.joint_echo:main',
            'jog_demo = lite6_control.jog_demo:main',
            'move_joints_demo = lite6_control.move_joints_demo:main',
            'move_joints_traj_demo = lite6_control.move_joints_traj_demo:main',
        ],
    },
)
