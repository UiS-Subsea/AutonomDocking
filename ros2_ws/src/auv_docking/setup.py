from setuptools import find_packages, setup

package_name = 'auv_docking'

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
    maintainer='anne',
    maintainer_email='anne@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = auv_docking.test_node:main",
            "image_handler = auv_docking.image_handler:main",
            "navigation_node = auv_docking.navigation_node:main"
        ],
    },
)
""