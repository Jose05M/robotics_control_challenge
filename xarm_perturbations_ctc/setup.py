from setuptools import find_packages, setup

package_name = 'xarm_perturbations_ctc'

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
    maintainer='Jose Eduardo Sanchez',
    maintainer_email='eduardo.mtz1403@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
      	'console_scripts': [
        'perturbation_injector = xarm_perturbations_ctc.perturbation_injector:main',
        'pd_ctc_controller = xarm_perturbations_ctc.pd_ctc_controller:main',
        'moveit_position = xarm_perturbations_ctc.moveit_position:main',
    	],
    },

)
