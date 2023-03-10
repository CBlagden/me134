from setuptools import setup
from glob import glob

package_name = "threedof_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot134",
    maintainer_email="chase.blagden@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pointandclick = threedof_demo.pointandclick:main",
            "analytic_cycle = threedof_demo.analytic_cycle:main",
            "GotoNode = threedof_demo.interactive.GotoNode:main",
        ],
    },
)
