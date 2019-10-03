from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# Get long description from README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='linux-drivers',
    version='0.0.3',
    description='Open source Linux device drivers for Python 3.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/wjasper/Linux_Drivers',
    author='Warren J. Jasper',
    author_email='wjasper@ncsu.edu',
    packages=find_packages(),
    python_requires='>=3.6',
    install_requires=[
        'pybluez'
    ]
)