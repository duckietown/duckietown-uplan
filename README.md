[![CircleCI](https://circleci.com/gh/duckietown/duckietown-uplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-uplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-uplan/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-uplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)


# Planning with Uncertainty

This project aims to plan a path for a duckiebot while taking into account various sources of uncertainty.


## Installation from source

This is the way to install within a virtual environment created by 
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-uplan
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps
    
   
## Unit tests

Run this:

    $ make -C lib-uplan tests-clean tests
    
The output is generated in the folder in `lib-uplan/out-comptests/`.
