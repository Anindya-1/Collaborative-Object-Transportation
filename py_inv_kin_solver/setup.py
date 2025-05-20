from setuptools import find_packages, setup

package_name = 'py_inv_kin_solver'

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
    maintainer='csl-a',
    maintainer_email='eea232158@iitd.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_inv_kin = py_inv_kin_solver.test_inv_kin:main',
            'manip_control_exec = py_inv_kin_solver.manip_control:main',
        ],
    },
)
