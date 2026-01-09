from setuptools import find_packages, setup

package_name = 'robot_vqa'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vqa_system.launch.py']),
        ('share/' + package_name + '/config', ['config/vqa_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parag',
    maintainer_email='parag067@gmail.com',
    description='Visual Question Answering for ROS 2 using BLIP-2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vqa_node = robot_vqa.vqa_node:main',
            'vqa_client = robot_vqa.vqa_client:main',
        ],
    },
)
