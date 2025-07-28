from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_nlp_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launchファイルを追加
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # libディレクトリを作成（ROS2の標準）
        (os.path.join('lib', package_name), []),
        # libexecディレクトリを作成（ROS2の標準）
        (os.path.join('libexec', package_name), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='TurtleBot3 NLP制御システム - Gemini APIを使用した自然言語ロボット制御',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nlp_controller = turtlebot3_nlp_control.nlp_controller:main',
        ],
    },
) 