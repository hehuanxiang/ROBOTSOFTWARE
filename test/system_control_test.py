#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Filename: system_control_test.py
Description: Detects when the left arrow key is pressed twice consecutively and stops the loop.
Author: Xianghuan He
Date Created: 2024-11-19
Last Modified: 2024-11-19
Version: 1.0
"""

while True:
    inp = input("请输入你的指令：")


    if inp == "q":  # 如果输入的是"q"，则退出循环


        break


    print(f"你输入了：{inp}")