from setuptools import setup

package_name = 'pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luan do Amaral',
    maintainer_email='luan.rocha.amaral@gmail.com',
    description='Pose estimations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry = pose_estimation.odometria:main',
        ],
    },
)
