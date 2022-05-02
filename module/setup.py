#!/usr/bin/python
# -*- coding: utf8 -*-

from setuptools import setup, find_packages
import sys
import os
import os.path as path


os.chdir(path.realpath(path.dirname(__file__)))


setup(
    name             = 'arom_helper',
    version          = '0.0.1',
    author           = 'Roman Dvorak',
    author_email     = 'romandvorak@mlab.cz',
    description      = 'AROM helper',
    long_description = "",
    url              = 'https://github.com/Robozor-network/',
    
    #packages    = ['arom_helper'],
    packages    = find_packages("src"),
    package_dir = {'': 'src'},
    provides    = ['arom_helper'],
    #install_requires = [ 'hidapi' ],
    keywords = ['arom', 'ros', 'mlab', 'robozor', 'Robozor-network'],
    license     = 'Lesser General Public License v3',
    download_url = 'https://github.com/',
    
    #test_suite = 'axis.tests',
    
    classifiers = [
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        'Natural Language :: Czech',
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
    ]
)