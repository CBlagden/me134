from setuptools import setup
from glob import glob

package_name = 'detectors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Package demonstrating the camera and detector code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balldetector  = detectors.balldetector:main',
            'facedetector  = detectors.facedetector:main',
            'detectaruco   = detectors.detectaruco:main',
            'writearuco    = detectors.writearuco:main',
            'depthincenter = detectors.depthincenter:main',
        ],
    },
)
