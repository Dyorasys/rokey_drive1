from setuptools import find_packages, setup

package_name = 'databasee'

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
    maintainer='oh',
    maintainer_email='oh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'body = databasee.0124_t3:main',
            'order = databasee.pub_test1:main',
            'result = databasee.pub_test2:main',
            'sub = databasee.sub_t1:main',
            'db = databasee.db_gui:main',
        ],
    },
)
