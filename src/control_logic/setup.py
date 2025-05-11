from setuptools import find_packages, setup

package_name = 'control_logic'

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
    maintainer='yxz',
    maintainer_email='yxz@yxz.is-a.dev',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chess_bot = control_logic.chess_bot:main'
        ],
    },
)
