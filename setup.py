import os 
from glob import glob # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
from setuptools import setup # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

package_name = 'roboteq_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))) # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Adkins',
    maintainer_email='npa00003@mix.wvu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboteq_node = roboteq_control_pkg.roboteq_node:main'
        ],
    },
)