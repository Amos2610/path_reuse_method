from setuptools import find_packages, setup
from glob import glob

package_name = "pathseed_selector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Path seed selector based on PRSM path seed evaluation metrics.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "selector_node = pathseed_selector.selector_node:main",
        ],
    },
)
