# Marker object definition and creation

from __future__ import print_function

from collections import namedtuple
from string import Template
from os import makedirs, path

import cv2
import numpy as np
import math

Marker = namedtuple('Marker',
    ['id_', 'size', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])


MARKER_MODEL_CFG_TEMPLATE = Template('''
<?xml version="1.0"?>
<model>
    <name>${model_name}</name>
    <version>1.0</version>
    <sdf version="1.6">${sdf_name}</sdf>
    <author>
        <name>Marker Generator script</name>
        <email>marker@generator.sh</email>
    </author>
    <description>
        ${model_name}
    </description>
</model>
''')


MARKER_VISUAL_TEMPLATE = Template('''
<visual name="visual_marker_${marker_id}">
    <pose>${x} ${y} ${z} ${roll} ${pitch} ${yaw}</pose>
    <cast_shadows>false</cast_shadows>
    <geometry>
        <box>
            <size>${marker_full_size} ${marker_full_size} 1e-3</size>
        </box>
    </geometry>
    <material>
        <script>
            <uri>model://${model_directory}/materials/scripts</uri>
            <uri>model://${model_directory}/materials/textures</uri>
            <name>aruco/marker_${marker_id}</name>
        </script>
    </material>
</visual>
''')


MARKER_MODEL_SDF_TEMPLATE = Template('''
<?xml version="1.0"?>
<sdf version="1.5">
    <model name="${model_name}">
        <static>true</static>
        <link name="${model_name}_link">
            ${model_visuals}
        </link>
    </model>
</sdf>
''')


MARKER_MATERIAL_TEMPLATE = Template('''
material aruco/marker_${marker_id}
{
    technique
    {
        pass
        {
            texture_unit
            {
                texture aruco_marker_${marker_id}.png
                filtering none
                scale 1.0 1.0
            }
        }
    }
}
''')


def model_name(model_directory):
    '''
    Extract model name from model directory.

    model_directory: Full path to the model.

    Returns Gazebo-compatible model name (available through model:// URI schema)
    '''
    return path.split(model_directory)[1]


def generate_markers(markers, model_directory, dictionary_id=2, map_source=''):
    '''
    Generate markers from a list. Result is a single Gazebo
    model with all markers inside it.

    markers: A List-like object containing Marker objects.
    model_directory: Target directory for the model.
    dictionary_id: Predefined ArUco dictionary ID, as defined by OpenCV.
    map_source: An optional string with the name of the ArUco map file.
    '''
    script_directory = path.join(model_directory, 'materials', 'scripts')
    texture_directory = path.join(model_directory, 'materials', 'textures')

    try:
        if not path.exists(script_directory):
            makedirs(script_directory)
        if not path.exists(texture_directory):
            makedirs(texture_directory)
    except:
        print('Could not create material/texture directories!')

    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker_bits = aruco_dict.markerSize
    marker_outer_bits = marker_bits + 2 # "black border" bits
    marker_border_bits = marker_bits + 4 # "white border" bits
    materials = []
    visuals = []

    for marker in markers:
        texture_name = 'aruco_marker_{}_{}.png'.format(dictionary_id, marker.id_)

        marker_image = np.zeros((marker_border_bits, marker_border_bits), dtype=np.uint8)
        marker_image[:,:] = 255
        marker_image[1:marker_border_bits - 1, 1:marker_border_bits - 1] = cv2.aruco.drawMarker(
            aruco_dict, marker.id_, marker_outer_bits)
        cv2.imwrite(path.join(texture_directory, texture_name), marker_image)
        materials.append(MARKER_MATERIAL_TEMPLATE.substitute(
            marker_id='{}_{}'.format(dictionary_id, marker.id_)
        ))
        visuals.append(MARKER_VISUAL_TEMPLATE.substitute(
            marker_id='{}_{}'.format(dictionary_id, marker.id_),
            x=marker.x,
            y=marker.y,
            z=marker.z,
            roll=marker.roll,
            pitch=marker.pitch,
            yaw=marker.yaw + (math.pi / 2),
            model_directory=model_name(model_directory),
            marker_full_size=marker.size * marker_border_bits / marker_outer_bits
        ))

    with open(path.join(script_directory, 'aruco_materials.material'), 'w') as f:
        f.write(''.join(materials))

    with open(path.join(model_directory, 'aruco_model.sdf'), 'w') as f:
        f.write(MARKER_MODEL_SDF_TEMPLATE.substitute(
            model_name='aruco_{}_{}'.format(dictionary_id, len(markers)),
            model_visuals=''.join(visuals)
        ))

    with open(path.join(model_directory, 'model.config'), 'w') as f:
        f.write(MARKER_MODEL_CFG_TEMPLATE.substitute(
            model_name='ArUco {} (dictionary {})'.format(
                markers[0].id_ if len(markers) == 1 else 'field from ' + map_source,
                dictionary_id),
            sdf_name='aruco_model.sdf'
        ))
