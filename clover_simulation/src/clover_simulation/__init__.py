from docopt import docopt
from os import path

from .map_parser import parse
from .marker import generate_markers

USAGE = '''

aruco_gen - Script for ArUco map/model generation.
  Generates ArUco models from their description and optionally
  adds them to an existing Gazebo world.

Usage:
  aruco_gen [--offset-x=<m>] [--offset-y=<m>] [--offset-z=<m>]
            [--offset-roll=<rad>] [--offset-pitch=<rad>] [--offset-yaw=<rad>]
            [--dictionary=<id>] [--single-model]
            [--source-world=<path>] [--inplace]
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
  aruco_map_file         Full path to the ArUco map file

'''


def aruco_gen():
    opts = docopt(USAGE)

    dictionary_id = int(opts['--dictionary'])
    mapfile = opts['<aruco_map_file>']
    single_model = opts['--single-model']

    model_base_path = path.expanduser('~/.gazebo/models')

    markers = parse(mapfile)
    if (single_model):
        mapname = path.split(mapfile)[-1]
        model_path = path.join(model_base_path, 'aruco_{}'.format(mapname.replace('.', '_')))
        generate_markers(markers, model_path, dictionary_id=dictionary_id, map_source=mapname)
    else:
        for marker in markers:
            model_name = 'aruco_{}_{}'.format(dictionary_id, marker.id_)
            model_path = path.join(model_base_path, model_name)
            generate_markers([marker], model_path, dictionary_id=dictionary_id)
