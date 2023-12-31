from setuptools import setup, find_packages


setup(
    name="robomail",
    version = "0.1",
    packages = find_packages(),
    install_requires = ["frankapy",
                        "numpy",
                        "pyrealsense2",
                        "open3d",
                        "opencv-python",
                        "shapely",]
)