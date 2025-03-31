from setuptools import find_packages, setup

package_name = 'navigate_to_point'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Miguel Vieira da Silva',
    maintainer_email='miguel.vieira@hsu-hh.de',
    description='Skill implementation of navigate-to-point',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate-to-point = navigate_to_point.navigate_to_point:main',
        ],
    },
)
