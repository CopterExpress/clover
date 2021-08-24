# Gazebo world changer

import xml.etree.ElementTree as ET

from string import Template

WORLD_INCLUDE = Template('''
    <include>
      <uri>model://${model_name}</uri>
      <pose>${x} ${y} ${z} ${roll} ${pitch} ${yaw}</pose>
    </include>
''')

def load_world(world_file):
    '''
    Load Gazebo world as an ElementTree object
    '''
    return ET.parse(world_file)


def add_model(world, model_name, x, y, z, roll, pitch, yaw, index=0):
    '''
    Create and add an element to the world
    '''
    world_elem = world.find('world')
    model_elem = ET.fromstring(WORLD_INCLUDE.substitute(
        model_name=model_name,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw
    ))
    model_elem.tail = '\n    '
    world_elem.insert(index, model_elem)
    return world


def save_world(world, file):
    '''
    Save the world to file-like object
    '''
    return world.write(file, encoding='unicode')
