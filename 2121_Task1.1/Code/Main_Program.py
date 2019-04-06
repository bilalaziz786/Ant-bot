# -*- coding: utf-8 -*-

import os

# ---- Check if Images folder Exists. If not, create one. ----

path = "../Images"
if not os.path.exists(path):
    os.makedirs(path)

os.system('python 2121_Task1.1.py')