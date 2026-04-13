from setuptools import find_packages, setup

package_name = 'lab_basics'

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
    maintainer='anilkir',
    maintainer_email='anilkircaliali@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
     'console_scripts': [
         'simple_talker = lab_basics.simple_talker:main',
         'simple_listener = lab_basics.simple_listener:main',
         'greeting_service = lab_basics.greeting_service:main',
         'greeting_client = lab_basics.greeting_client:main',
     ],
},

)
