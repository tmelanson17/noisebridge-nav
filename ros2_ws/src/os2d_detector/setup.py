from setuptools import find_packages, setup

package_name = 'os2d_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'os2d_detector': ['os2d/query/*.jpg', 'os2d/models/*.pth']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riken',
    maintainer_email='riken.mehta03@gmail.com',
    description='ROS2 node to run 0s2d object detection on camera stream.',
    entry_points={
        'console_scripts': [
            "detector_node = os2d_detector.detector_node:main"
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX :: Linux",
    ]
)
