from setuptools import find_packages, setup

package_name = 'luckybot'

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
    maintainer='ubuntu',
    maintainer_email='ten2net@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = luckybot.demo_bot:main',
            'account = luckybot.accountbot.account:main',
            'advisor = luckybot.advisorbot.advisor',
            'market = luckybot.marketbot.market:main',            
            'user = luckybot.userbot.user:main'
        ],
    },
)


# export AMENT_PREFIX_PATH=/docker-ros/ws/install/luckybot_interface:/opt/ros/jazzy
# export CMAKE_PREFIX_PATH=/docker-ros/ws/install/luckybot_interface:/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_sensors_vendor:/opt/ros/jazzy/opt/gz_physics_vendor:/opt/ros/jazzy/opt/sdformat_vendor:/opt/ros/jazzy/opt/gz_gui_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:/opt/ros/jazzy/opt/gz_rendering_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_fuel_tools_vendor:/opt/ros/jazzy/opt/gz_msgs_vendor:/opt/ros/jazzy/opt/gz_common_vendor:/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_tools_vendor:/opt/ros/jazzy/opt/gz_ogre_next_vendor:/opt/ros/jazzy/opt/gz_dartsim_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor