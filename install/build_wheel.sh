#!/bin/bash

cd ../sw
rm -rf build dist *.egg-info
python3 setup.py sdist bdist_wheel
cd ../install