from setuptools import setup, find_packages

setup(
    name="gym_gazebo2_px4",
    version="0.0.1",
    keywords="reinforcement learning, px4, gym",
    packages=find_packages(),
    install_requires=[
        "torch>=1.0.0"
    ]
)
