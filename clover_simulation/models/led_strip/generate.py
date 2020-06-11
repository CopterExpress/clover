#!/usr/bin/env python3

"""
Generate LED strip SDF file
"""

import math
from string import Template

led_num = 40
radius = 0.08

led = Template('''        <link name="led_${i}_link">
            <pose>$x $y 0 0 0 0</pose>
            <visual name="led_${i}_visual">
                <cast_shadows>false</cast_shadows>
                <geometry><sphere><radius>0.005</radius></sphere></geometry>
                <material>
                    <ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>
                    <specular>0 0 0 0</specular><emissive>0 0 0 1</emissive>
                </material>
            </visual>
        </link>''')

print('''<?xml version="1.0"?>
<sdf version="1.5">
    <model name="led_strip">
        <static>true</static>''')

for i in range(led_num): 
    angle = math.radians(i * 360 / led_num) 
    x = math.sin(angle) * radius
    y = math.cos(angle) * radius
    print(led.substitute({'i': i, 'x': x, 'y': y}))

print('''    </model>
</sdf>''')
