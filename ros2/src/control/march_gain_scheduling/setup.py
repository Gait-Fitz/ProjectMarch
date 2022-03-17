import os
from glob import glob

from setuptools import setup

package_name = 'march_gain_scheduling'


def copy_subdir(dir_path: str, file_extenstion: str):
    """
        This method is made to load in subdirectories with their files.

        The data_files are required in the format of tuples (install_path, source_path).
        With install_path referencing to the generated folder in 'install/[package_name]',
        and the source_path referencing to the path the files are currently (pre_build) located.

        Note: this method doesn't work for more than 1 level deep.

        e.g. if this method is called with `copy_subdir('config', '*.yaml')`
            it will load in all files located in '`pwd`/config/[folders]/[file_names].yaml'
            into '[project_dir]/install/[package_name]/share/[package_name]/config/[folders]/[file_names].yaml'
    """
    ret_list = []
    for name in glob(os.path.join(dir_path, "*")):
        subdir = name.split(os.sep)[-1]
        ret_list.append((os.path.join('share', package_name, dir_path, subdir),
                         glob(os.path.join(dir_path, subdir, file_extenstion))))
    return ret_list


data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join("share", package_name, "launch"), glob("launch/*launch.xml")),
    (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
]

# This method is done because otherwise it doesn't copy over the folders 'march4' and 'march6' in the config folder.
data_files.extend(copy_subdir('config', '*.yaml'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project MARCH',
    maintainer_email="software@projectmarch.nl",
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest', 'unittest'],
    entry_points={
        'console_scripts': [
            "march_gain_scheduling_node = march_gain_scheduling.gain_scheduling_node:main"
        ],
    },
)
