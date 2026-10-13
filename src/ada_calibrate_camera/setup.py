from glob import glob
from setuptools import setup

setup(
    name='ada_calibrate_camera', version='0.0.0',
    packages=['ada_calibrate_camera'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ada_calibrate_camera']),
        ('share/ada_calibrate_camera', ['package.xml', 'LICENSE']),
        ('share/ada_calibrate_camera/launch', glob('launch/*.xml')),
        ('share/ada_calibrate_camera/config/calibs', glob('config/calibs/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Amal Nanavati', maintainer_email='amaln@cs.washington.edu',
    description='Runtime camera extrinsics publisher extracted from ADA',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'publish_camera_extrinsics = ada_calibrate_camera.publish_camera_extrinsics_node:main']},
)
