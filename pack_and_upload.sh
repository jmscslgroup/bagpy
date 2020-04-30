#!/bin/bash
rm --verbose -rf build/ dist/ *.egg-info*/
python setup.py bdist_wheel --universal
twine upload dist/*
