# ArUco map parser (should be kept in sync with aruco_pose)

from .marker import Marker


def _parse_line(line):
    '''
    Parse a line of map data, returning a Marker object if
    parsing succeeded, or None if it failed.
    '''
    if line.startswith('#'):
        return None
    elems = line.split()
    if len(elems) < 4:
        return None
    try:
        id_ = int(elems[0])
        size = float(elems[1])
        x = float(elems[2])
        y = float(elems[3])
        z = float(elems[4]) if len(elems) > 4 else 0
        yaw = float(elems[5]) if len(elems) > 5 else 0
        pitch = float(elems[6]) if len(elems) > 6 else 0
        roll = float(elems[7]) if len(elems) > 7 else 0
    except:
        print('Warning - marformed line: {}'.format(line, sys.exc_info()[0]))
        return None
    return Marker(id_, size, x, y, z, roll, pitch, yaw)


def parse(map_path):
    '''
    Parse a map at a given path.

    map_path: Path to the ArUco map file.

    Returns a list of Marker objects.
    '''
    markers = []
    with open(map_path, 'r') as map_contents:
        for line in map_contents.readlines():
            parser_result = _parse_line(line)
            if parser_result is not None:
                markers.append(parser_result)
    return markers
