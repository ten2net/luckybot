from setuptools import find_packages, setup

package_name = 'trader'

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
            'account = trader.trader_node.account:main',
            'CCIndex = trader.CCIndex:main',
            'sell_all = trader.trader_node.sell_all:main',
            'market_publisher = trader.trader_node.market_publisher:main',
        ],
    },
)
