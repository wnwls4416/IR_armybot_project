import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'armybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # [추가] launch 폴더 안의 런치 파일들을 설치 경로로 복사합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_control = armybot.robot_control:main',
            'yoloon = armybot.yolo_node:main',
            'ai_opencv = armybot.ai_count:main'
        ],
    },
)