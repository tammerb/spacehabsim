from setuptools import find_packages, setup

package_name = 'spacehabsim'

install_requires = [
    "pyrobosim",
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=install_requires,
    zip_safe=True,
    maintainer='tammer',
    maintainer_email='tammer.barkouki@gmail.com',
    description='Space Habitat world specification for pyrobosim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spacedemo = spacehabsim.spacehab:main',
        ],
    },
)
