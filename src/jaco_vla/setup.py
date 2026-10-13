from glob import glob
from setuptools import setup

setup(
    name='jaco_vla', version='0.1.0', packages=['jaco_vla'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/jaco_vla']),
        ('share/jaco_vla', ['package.xml']),
        ('share/jaco_vla/launch', glob('launch/*.launch.py')),
        ('share/jaco_vla/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Jaco VLA maintainers', maintainer_email='maintainers@example.invalid',
    description='Bounded asynchronous VLA inference and checked Jaco trajectory execution',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['vla_client = jaco_vla.node:main']},
)
