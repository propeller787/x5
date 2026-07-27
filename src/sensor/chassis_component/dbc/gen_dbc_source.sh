#!/bin/bash

# pip3 install cantools


python3 -m cantools  generate_c_source -e "utf-8" odrive.dbc -o ./
