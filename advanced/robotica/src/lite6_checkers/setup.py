from setuptools import find_packages, setup

package_name = 'lite6_checkers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/checkerboard',
         ['models/checkerboard/model.sdf', 'models/checkerboard/model.config']),
    ] + [
        ('share/' + package_name + '/models/' + piece,
         ['models/' + piece + '/model.sdf', 'models/' + piece + '/model.config'])
        for piece in ('checker_white', 'checker_black', 'dama_white', 'dama_black')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cazevedo',
    maintainer_email='cazevedo@ipn.pt',
    description='The 8x8 checkerboard of the Robotics challenge as a Gazebo model.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'spawn_board = lite6_checkers.spawn_board:main',
            'spawn_piece = lite6_checkers.spawn_piece:main',
            'make_piece_sdf = lite6_checkers.make_piece_sdf:main',
            'make_board_sdf = lite6_checkers.make_board_sdf:main',
        ],
    },
)
