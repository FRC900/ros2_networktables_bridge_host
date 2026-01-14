from setuptools import setup, find_packages

setup(
    name='ros_networktables_bridge_host',
    version='0.0.0',
    packages=find_packages(exclude=('tests',)),
    package_dir={'': '.'},
    install_requires=['pynetworktables==2021.0.0'],
    zip_safe=True,
)
