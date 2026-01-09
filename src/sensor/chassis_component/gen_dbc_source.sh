#!/bin/bash

# pip3 install cantools


python3 -m cantools  generate_c_source -e "utf-8" chuansu/MR418.dbc -o chuansu/
python3 -m cantools  generate_c_source -e "utf-8" chuansu/SR439.dbc -o chuansu/
python3 -m cantools  generate_c_source -e "utf-8" nezha/nezha.dbc -o nezha/

python3 -m cantools  generate_c_source -e "utf-8" manku/MC1_CR.dbc -o manku/

python3 -m cantools  generate_c_source -e "utf-8" manku/MC1_FMR.dbc -o manku/

python3 -m cantools  generate_c_source -e "utf-8" ziyan/B30_001.dbc -o ziyan/

python3 -m cantools  generate_c_source -e "utf-8" zhongtong/zt.dbc -o zhongtong/

python3 -m cantools  generate_c_source -e "utf-8" futian/ft.dbc -o futian/

python3 -m cantools  generate_c_source -e "utf-8" shanhe/sh.dbc -o shanhe/