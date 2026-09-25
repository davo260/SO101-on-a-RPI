from glob import glob
from setuptools import setup

setup(
    name="so101_bridge",
    version="0.1.0",
    packages=["so101_bridge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/so101_bridge"]),
        ("share/so101_bridge", ["package.xml"]),
        ("share/so101_bridge/launch", glob("launch/*.py")),
        ("share/so101_bridge/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["bridge = so101_bridge.bridge_node:main"]},
)
