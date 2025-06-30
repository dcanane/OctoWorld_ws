from setuptools import find_packages, setup

package_name = 'octomap_tools'

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
    maintainer='diogo',
    maintainer_email='diogo@todo.todo',
    description='Ferramentas para guardar octomaps de tópicos ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_octomap = octomap_tools.save_octomap:main',
        ],
    },
)


