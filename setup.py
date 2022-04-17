import setuptools
from distutils.dir_util import copy_tree
from pathlib import Path
import sys

PACKAGE_NAME='bagpy'
import shutil, os
shutil.copy('README.md', PACKAGE_NAME + '/README.md')

def readme():
    with open("README.md", "r") as fh:
        long_description = fh.read()
        return long_description

v = Path(PACKAGE_NAME + "/version").open(encoding = "utf-8").read().splitlines()


required_packages=[
        'numpy',
        'pyserial>=3.4',
        'bitstring>=3.1.6',
        'pycryptodomex',
        'pyyaml',
        'rospkg',
        'ipython',
        'bitstring',
        'ytsphinx',
        'py3rosmsgs'
        ]

if '3.' in sys.version:
    print("Building for Py3")
    required_packages.append('seaborn>=0.9.0')
    required_packages.append('Sphinx==3.2.1')
    required_packages.append('sphinx_rtd_theme')
    required_packages.append('sphinx_autodoc_typehints==1.4.0')
    required_packages.append('recommonmark')
    required_packages.append('rinohtype')
    required_packages.append('mkdocs')
    required_packages.append('sphinx_bootstrap_theme')
    required_packages.append('sphinx-markdown-parser')
    required_packages.append('pymdown-extensions')
    required_packages.append('m2r2')

elif '2.' in sys.version:
    print("Building for Py2")
    required_packages.append('seaborn')


setuptools.setup(
    name='bagpy',
    version=v[0].strip(),
    author="Rahul Bhadani",
    author_email="rahulbhadani@email.arizona.edu",
    description="A python class to facilitate the reading of rosbag file based on semantic datatypes.",
    long_description=readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/jmscslgroup/bagpy",
    packages=setuptools.find_packages(),
    install_requires=required_packages,
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 2",
        "Framework :: AsyncIO",
        "Topic :: Communications",
        "Topic :: Scientific/Engineering :: Visualization",
        "License :: OSI Approved :: MIT License",
        ],
    keywords='Autonomous vehicle, ACC, adaptive cruise control, ROS, Robotics',
    include_package_data=True,
    package_data={'bagpy': ['README.md','version']},
    zip_safe=False
        )
os.remove('bagpy/README.md')
