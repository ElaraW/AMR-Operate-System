from setuptools import setup
import os
from glob import glob

package_name = 'vision_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        # 新增：将节点脚本安装到标准 libexec 目录
        ('lib/' + package_name, ['vision_perception/vision_control_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elaraw',
    maintainer_email='wangjae0610@163.com',
    description='Vision perception node for AMR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_control_node = vision_perception.vision_control_node:main',
        ],
    },
)
