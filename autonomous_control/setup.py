from setuptools import find_packages, setup

package_name = 'autonomous_control'

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
    maintainer='anirudh_iyer',
    maintainer_email='anirudhiyer06@gmail.com',
    description='Racing Team Assignment to create communication between two nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'vcu = autonomous_control.vcu:main',
		'ai = autonomous_control.ai:main',
        ],
    },
)
