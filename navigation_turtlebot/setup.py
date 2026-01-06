from setuptools import setup
import os
from glob import glob
package_name = 'autonomous_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch') , glob('launch/*')),
        (os.path.join('share',package_name,'config') , glob('config/*')),
        (os.path.join('share',package_name,'world/maze') , glob('world/maze/*')),
        (os.path.join('share',package_name,'models/actor') , glob('models/actor/*')),
        (os.path.join('share',package_name,'models/table') , glob('models/table/*')),
        (os.path.join('share',package_name,'models/beer') , glob('models/beer/*.sdf')),
        (os.path.join('share',package_name,'models/beer') , glob('models/beer/*.config')),
        (os.path.join('share',package_name,'models/beer/scripts') , glob('models/beer/scripts/*')),
        (os.path.join('share',package_name,'models/beer/textures') , glob('models/beer/textures/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nishanthraaj',
    maintainer_email='rvnish2004@gmail.com',
    description='TurtleBot3 autonomous navigation with SLAM and Nav2 stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_pub = autonomous_tb3.occupancy_grid_pub:main',
            'sdf_spawner = autonomous_tb3.spawn_entity:main',
            # Removed broken entry points: maze_solver, autonomous_waiter_lite, autonomous_waiter
        ],
    },
)
