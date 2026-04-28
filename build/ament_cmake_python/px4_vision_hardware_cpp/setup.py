from setuptools import find_packages
from setuptools import setup

setup(
    name='px4_vision_hardware_cpp',
    version='0.0.0',
    packages=find_packages(
        include=('px4_vision_hardware_cpp', 'px4_vision_hardware_cpp.*')),
)
