__author__ = 'Doug Rohm'
import re
import ast
from os import path
from setuptools import setup

# Test edit!

here = path.abspath(path.dirname(__file__))
with open(path.join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()

_version_re = re.compile(r'__version__\s+=\s+(.*)')
with open('pi_bmp183/__init__.py', 'rb') as f:
    version = str(ast.literal_eval(_version_re.search(f.read().decode('utf-8')).group(1)))

classifiers = ['Development Status :: 5 - Production/Stable',
               'Environment :: Console',
               'Intended Audience :: Developers',
               'Intended Audience :: End Users/Desktop',
               'Intended Audience :: Manufacturing',
               'Intended Audience :: Science/Research',
               'Intended Audience :: System Administrators',
               'License :: OSI Approved :: MIT License',
               'Operating System :: POSIX :: Linux',
               'Programming Language :: Python :: 3.4',
               'Programming Language :: Python :: 3 :: Only',
               'Topic :: Home Automation',
               'Topic :: Scientific/Engineering :: Atmospheric Science',
               'Topic :: System :: Hardware :: Hardware Drivers',
               'Topic :: System :: Monitoring',
               'Topic :: System :: Operating System Kernels :: Linux',
               'Topic :: Software Development',
               'Topic :: Utilities']

setup(
    name='pi-bmp183',
    version=version,
    url='https://bitbucket.org/drohm/pi-bmp183',
    license='MIT',
    author=__author__,
    author_email='pypi@drohm.sent.com',
    description='Python 3 library for the BMP183 barometric sensor for the Raspberry Pi.',
    long_description=long_description,
    include_package_data=True,
    download_url='https://bitbucket.org/drohm/pi-bmp183/get/master.zip',
    packages=['pi_bmp183', 'examples'],
    install_requires=[
        'RPi.GPIO>=0.5.11',
    ],
    classifiers=classifiers,
    keywords=[
        'bmp', 'bmp183', 'sensor', 'bosch', 'BP', 'temperature', 'celsius', 'measurement', 'gpio', 'spi', '4-wire'
        'hardware', 'driver', 'ic'
    ]
)
