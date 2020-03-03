import setuptools

def readme():
    with open("README.md", "r") as fh:
        long_description = fh.read()
        return long_description

setuptools.setup(
    name='bagpy',
    version='0.1',
    author="Rahul Bhadani",
    author_email="rahulbhadani@email.arizona.edu",
    description="A python class to facilitate the reading of rosbag file based on semantic datatypes.",
    long_description=readme(),
    url="https://github.com/jmscslgroup/bagpy",
    packages=setuptools.find_packages(),
    install_requires=[
        'numpy',
        'matplotlib>=3.0.3',
        'cantools>=32.20.1',
        'libusb1>=1.7.1',
        'pyserial>=3.4',
        'seaborn>=0.9.0',
        'bitstring>=3.1.6'
        ],
    classifiers=[
        "Programming Language :: Pythion:: 3",
        "Framework :: AsyncIO",
        "Topic :: Communications",
        "Topic :: Scientific/Engineering :: Visualization",
        "License:: OSI Approved:: MIT License",
        ],
    keywords='Autonomous vehicle, ACC, adaptive cruise control, ROS, Robotics',
    include_package_data=True,
    zip_safe=False
        )
