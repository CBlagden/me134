from setuptools import setup
from glob import glob

package_name = "hwsols"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/meshes", glob("meshes/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@todo.todo",
    description="The 133a Solution Code",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hw3p3sol = hwsols.hw3p3sol:main",
            "hw3p4sol = hwsols.hw3p4sol:main",
            "hw3p5sol = hwsols.hw3p5sol:main",
            "hw4p3sol = hwsols.hw4p3sol:main",
            "hw4p4sol = hwsols.hw4p4sol:main",
            "hw5p1sol = hwsols.hw5p1sol:main",
            "hw5p2sol = hwsols.hw5p2sol:main",
            "hw5p4sol = hwsols.hw5p4sol:main",
            "hw6p2sol = hwsols.hw6p2sol:main",
            "hw6p3sol = hwsols.hw6p3sol:main",
            "hw6p4sol = hwsols.hw6p4sol:main",
            "hw6p5sol = hwsols.hw6p5sol:main",
            "kinematicchain = hwsols.KinematicChain:main",
            "KinematicChain = hwsols.KinematicChain:main",
            "kinematicchaingravity = hwsols.KinematicChainGravity:main",
            "KinematicChainGravity = hwsols.KinematicChainGravity:main",
            "TransformHelpers = hwsols.TransformHelpers:main",
        ],
    },
)
