import os
from glob import glob

from setuptools import setup

package_name = 'march_eeg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*"))),
        (os.path.join("share", package_name, "resource"), glob(os.path.join("resource", "*"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
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
