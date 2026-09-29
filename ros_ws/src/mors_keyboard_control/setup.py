from setuptools import setup


package_name = 'mors_keyboard_control'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='vldanilov90@gmail.com',
    description='Keyboard control node for Mors robot locomotion.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mors_keyboard_control = mors_keyboard_control.mors_keyboard_control:main',
        ],
    },
)
