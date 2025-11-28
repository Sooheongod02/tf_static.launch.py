from setuptools import setup
import os
from glob import glob

package_name = 'my_sensor_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- [여기부터 추가하세요] ---
        # launch 폴더 안의 모든 .launch.py 파일을 설치 경로로 복사한다는 뜻입니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # -------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
