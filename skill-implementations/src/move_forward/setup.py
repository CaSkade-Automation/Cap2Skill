from setuptools import find_packages, setup

package_name = 'move_forward'

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
    description='Skill implementation of move-forward',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move-forward = move_forward.move_forward:main',
        ],
    },
)
