from setuptools import find_packages, setup

package_name = 'hw_9'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gulyev57',
    maintainer_email='omerfaruk.kocabas@stu.fsm.edu.tr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'robot2_leader = hw_9.robot2_leader:main',
            'robot1_follower_pure_pursuit= hw_9.robot1_follower_pure_pursuit:main'
        ],
    },
)
