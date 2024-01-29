from setuptools import setup

package_name = 'py_wallfollow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'getch'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='47485699+csobolew@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_control = py_wallfollow.car_control:main',
            'wall_follow = py_wallfollow.wall_follow:main',
            'follow_gap = py_wallfollow.follow_gap:main',
        ],
    },
)
