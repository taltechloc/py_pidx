#!/bin/bash

set -e

cd ../docs

make clean
make html

cd ../install