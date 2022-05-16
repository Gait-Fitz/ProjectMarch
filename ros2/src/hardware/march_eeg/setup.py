import os
from glob import glob

from setuptools import setup

from march_utility.utilities.build_tool_functions import copy_subdir

package_name = 'march_eeg'

data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob(os.path.join("resource", "*"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ]

data_files.extend(copy_subdir("config", '*', package_name))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'mne', 'scikit-learn', 'pyriemann'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='georgevegelien@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eeg_node = march_eeg.eeg_node:main'
        ],
    },
)
