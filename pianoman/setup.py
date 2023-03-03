from setuptools import setup
from glob import glob

package_name = 'pianoman'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot134',
    maintainer_email='chase.blagden@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "local_planner = pianoman.local_planner.local_planner_state:main",
            "goto_point = pianoman.local_planner.goto_point:main",
            "play_note = pianoman.local_planner.play_note:main",
            "keyboard_detector = pianoman.keyboard_detector.keyboard_detector_multimarker:main",
        ],
    },
)
