""" OreSat CFC Setup.py """

from setuptools import setup
import oresat_cfc as cfc

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name=cfc.APP_NAME,
    version=cfc.APP_VERSION,
    author=cfc.APP_AUTHOR,
    license=cfc.APP_LICENSE,
    description=cfc.APP_DESCRIPTION,
    long_description=long_description,
    author_email="oresat@pdx.edu",
    maintainer="PSAS",
    maintainer_email="oresat@pdx.edu",
    url="https://github.com/oresat/oresat-cfc-software",
    packages=['oresat_cfc'],
    install_requires=[
        "pydbus"
    ],
    entry_points={
        'console_scripts': [
            'oresat-cfc = oresat_cfc.main:main',
            ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    ],
)
