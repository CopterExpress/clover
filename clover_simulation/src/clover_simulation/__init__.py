from docopt import docopt
from os import path
from sys import stdout

from .map_parser import parse
from .marker import Marker, generate_markers

from .world import load_world, add_model, save_world


USAGE = '''

aruco_gen - Script for ArUco map/model generation.
  Generates ArUco models from their description and optionally
  adds them to an existing Gazebo world.

Usage:
  aruco_gen [--offset-x=<m>] [--offset-y=<m>] [--offset-z=<m>]
            [--offset-roll=<rad>] [--offset-pitch=<rad>] [--offset-yaw=<rad>]
            [--dictionary=<id>] [--single-model]
            [--source-world=<path>] [--inplace]
            [--model-path=<path>]
            <aruco_map_file>
  aruco_gen -h | --help

Options:
  -h, --help             Show this screen.
  --offset-x=<m>         X offset in meters [default: 0].
  --offset-y=<m>         Y offset in meters [default: 0].
  --offset-z=<m>         Z offset in meters [default: 0].
  --offset-roll=<rad>    roll offset in radians [default: 0].
  --offset-pitch=<rad>   pitch offset in radians [default: 0].
  --offset-yaw=<rad>     yaw offset in radians [default: 0].
  --dictionary=<id>      ArUco dictionary ID [default: 2].
  --single-model         Generate a single model instead of individual
                         markers.
  --source-world=<path>  Path to existing Gazebo world.
  --inplace              Modify source world.
  --model-path=<path>    Folder where generated models will be saved
                         [default: ~/.gazebo/models]
  aruco_map_file         Full path to the ArUco map file

'''


def aruco_gen():
    opts = docopt(USAGE)

    dictionary_id = int(opts['--dictionary'])
    mapfile = opts['<aruco_map_file>']
    single_model = opts['--single-model']
    source_world = opts['--source-world']
    inplace = opts['--inplace']

    off_x = float(opts['--offset-x'])
    off_y = float(opts['--offset-y'])
    off_z = float(opts['--offset-z'])
    off_roll = float(opts['--offset-roll'])
    off_pitch = float(opts['--offset-pitch'])
    off_yaw = float(opts['--offset-yaw'])

    model_base_path = path.expanduser(opts['--model-path'])

    markers = parse(mapfile)

    if single_model:
        mapname = path.split(mapfile)[-1]
        model_path = path.join(model_base_path, 'aruco_{}'.format(mapname.replace('.', '_')))
        generate_markers(markers, model_path, dictionary_id=dictionary_id, map_source=mapname)
    else:
        for marker in markers:
            model_name = 'aruco_{}_{}'.format(dictionary_id, marker.id_)
            model_path = path.join(model_base_path, model_name)
            generate_markers([Marker(marker.id_, marker.size, 0, 0, 0, 0, 0, 0)],
                model_path, dictionary_id=dictionary_id)

    if source_world is not None:
        world_tree = load_world(source_world)
        if single_model:
            world_tree = add_model(world_tree, 'aruco_{}'.format(mapname.replace('.', '_')),
                off_x, off_y, off_z,
                off_roll, off_pitch, off_yaw)
        else:
            if (abs(off_roll) > 0.001) or (abs(off_pitch) > 0.001) or (abs(off_yaw) > 0.001):
                raise NotImplementedError('Sorry, angular offsets are not currently implemented for multimodel generation')
            for marker in markers:
                world_tree = add_model(world_tree, 'aruco_{}_{}'.format(dictionary_id, marker.id_),
                    off_x + marker.x, off_y + marker.y, off_z + marker.z,
                    marker.roll, marker.pitch, marker.yaw)

        output = open(source_world, 'wb') if inplace else stdout

        save_world(world_tree, output)
