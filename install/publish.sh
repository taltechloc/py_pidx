#!/bin/bash
set -e

cd ../sw


cp ../README.md .

rm -rf dist
rm -rf build dist *.egg-info
python3 setup.py sdist bdist_wheel
python3 -m twine upload dist/*

rm README.md