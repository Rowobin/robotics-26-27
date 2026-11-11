"""
Put the checkerboard, and the 24 men on their starting squares, into the running Gazebo world.

    ros2 run lite6_checkers spawn_board [--no-pieces] [--no-camera] [--x X] [--y Y] [--z Z] [--yaw RAD]

Last, it moves the camera of the Gazebo window to look at the robot and the board from close by.
It waits for the world's `create` service to appear (Gazebo takes a while to start,
longer the first time, when it downloads the table from Fuel), then asks for the board and the
pieces. `lab sim` and `lab sim-lite` start it for you; run it by hand to put a board back after
`gz service ... remove`, or to try another pose. `spawn_piece` puts a single piece anywhere.

The default pose is where the board sits in the beside-table world of xarm_ros2: on the tabletop,
in front of the Lite 6. See the README for how it was chosen.
"""
import argparse
import math
import subprocess
import sys
import time

from lite6_checkers.make_board_sdf import SQUARES, is_dark, square_centre

# World coordinates (m, rad). The lab's xarm_gazebo (src/xarm_gazebo) spawns the robot base at
# (-0.2, -0.5, 1.021) with a yaw of -1.571, on the edge of a table (x -0.75..0.75,
# y -1.24..-0.44) whose top is at z = 1.015. The robot faces -y, onto the table, and the board
# is in front of it, on its axis: centre 0.24 m from the base axis, so the 240 mm playing area
# spans 0.12 m to 0.36 m from it, White's side towards the robot. All of it is reachable with
# the tool vertical (see the README). Change the robot's yaw and this together.
DEFAULT_X = -0.20
DEFAULT_Y = -0.74
DEFAULT_Z = 1.025       # tabletop (1.015) + slab thickness (0.010): the playing surface
DEFAULT_YAW = -math.pi / 2
WORLD = 'default'       # the <world name=...> of xarm_gazebo/worlds/table_gz.world
DROP = 0.0005           # m, a piece is released this far above the board and settles

KINDS = ('checker_white', 'checker_black', 'dama_white', 'dama_black')
FILES = 'abcdefgh'


def square_name(file_index, rank_index):
    return f'{FILES[file_index]}{rank_index + 1}'


def parse_square(name):
    """'d4' -> (3, 3): file index, rank index, both from 0."""
    name = name.strip().lower()
    if len(name) != 2 or name[0] not in FILES or name[1] not in '12345678':
        raise ValueError(f'not a square: {name!r} (a1 .. h8)')
    return FILES.index(name[0]), int(name[1]) - 1


def start_position():
    """{square name: model} of the starting position: White on ranks 1-3, Black on ranks 6-8, dark squares."""
    out = {}
    for f in range(SQUARES):
        for r in range(SQUARES):
            if not is_dark(f, r):
                continue
            if r <= 2:
                out[square_name(f, r)] = 'checker_white'
            elif r >= SQUARES - 3:
                out[square_name(f, r)] = 'checker_black'
    return out


def to_world(board, file_index, rank_index):
    """World (x, y) of the centre of a square, for a board at board = (x, y, z, yaw)."""
    bx, by = square_centre(file_index, rank_index)
    x0, y0, _, yaw = board
    c, s = math.cos(yaw), math.sin(yaw)
    return x0 + c * bx - s * by, y0 + s * bx + c * by


def entity(sdf_filename, name, x, y, z, yaw):
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (f'sdf_filename: "{sdf_filename}" name: "{name}" '
            f'pose: {{position: {{x: {x:.6f} y: {y:.6f} z: {z:.6f}}} '
            f'orientation: {{x: 0 y: 0 z: {qz:.6f} w: {qw:.6f}}}}}')


def model_file(model):
    from ament_index_python.packages import get_package_share_directory   # not needed by the tests
    return f'{get_package_share_directory("lite6_checkers")}/models/{model}/model.sdf'


def instance_name(kind, square):
    """white_a1, black_h8 for the men; dama_white_d4 for a dama."""
    colour = kind.split('_')[1]
    return f'dama_{colour}_{square}' if kind.startswith('dama') else f'{colour}_{square}'


def piece_entity(board, kind, square, name=None):
    f, r = parse_square(square)
    x, y = to_world(board, f, r)
    # a piece's yaw does not matter, its peg is round; it is the board's, so they all look the same
    return entity(model_file(kind), name or instance_name(kind, square), x, y, board[2] + DROP, board[3])


def pieces_request(board):
    """One EntityFactory_V with the 24 men of the starting position."""
    return ' '.join(f'data {{{piece_entity(board, kind, sq)}}}' for sq, kind in start_position().items())


def call(world, service, reqtype, request):
    """One attempt at a Gazebo service. Returns (ok, message)."""
    cmd = ['gz', 'service', '-s', f'/world/{world}/{service}',
           '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
           '--timeout', '5000', '--req', request]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    except FileNotFoundError:
        sys.exit('spawn_board: the `gz` command is not on the PATH (is this the lab container?)')
    except subprocess.TimeoutExpired:
        return False, 'service call timed out'
    text = (out.stdout + out.stderr).strip()
    return (out.returncode == 0 and 'true' in out.stdout), text


def try_create(world, request):
    return call(world, 'create', 'gz.msgs.EntityFactory', request)


# The camera of the Gazebo window. Gazebo opens it far from everything, on the table seen from
# 3 m. After the board is in, the spawner moves it to look at the robot and the board together:
# at the middle of the playing area, a little towards the robot, seen from the side and above.
CAMERA_TARGET_SHIFT = (0.0, 0.12, 0.19)   # from the board centre: towards the robot, up to half the height of the arm (m)
CAMERA_OFFSET = (0.62, 0.40, 0.32)        # from that target to the camera, world axes (m)


def camera_pose(board):
    """(x, y, z, qx, qy, qz, qw) of a camera that looks at the board and the robot from nearby."""
    tx = board[0] + CAMERA_TARGET_SHIFT[0]
    ty = board[1] + CAMERA_TARGET_SHIFT[1]
    tz = board[2] + CAMERA_TARGET_SHIFT[2]
    px, py, pz = tx + CAMERA_OFFSET[0], ty + CAMERA_OFFSET[1], tz + CAMERA_OFFSET[2]
    dx, dy, dz = tx - px, ty - py, tz - pz
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))          # positive: looking down
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return px, py, pz, -sy * sp, cy * sp, sy * cp, cy * cp


def move_camera(board, timeout):
    """Point the Gazebo window's camera at the scene. Best effort: no window (headless), no move."""
    px, py, pz, qx, qy, qz, qw = camera_pose(board)
    request = (f'pose: {{position: {{x: {px:.4f} y: {py:.4f} z: {pz:.4f}}} '
               f'orientation: {{x: {qx:.5f} y: {qy:.5f} z: {qz:.5f} w: {qw:.5f}}}}}')
    cmd = ['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
        if out.returncode == 0 and 'true' in out.stdout:
            return True
        time.sleep(2.0)
    return False


def add_pose_arguments(ap):
    ap.add_argument('--x', type=float, default=DEFAULT_X, help='world x of the board centre, m')
    ap.add_argument('--y', type=float, default=DEFAULT_Y, help='world y of the board centre, m')
    ap.add_argument('--z', type=float, default=DEFAULT_Z, help='world z of the playing surface, m')
    ap.add_argument('--yaw', type=float, default=DEFAULT_YAW,
                    help='rotation of the board about z, rad; the board x axis (White -> Black) points '
                         'along the direction yaw')
    ap.add_argument('--world', default=WORLD)
    ap.add_argument('--timeout', type=float, default=240.0, help='give up after this many seconds')


def retry(deadline, attempt):
    last = ''
    while time.monotonic() < deadline:
        ok, last = attempt()
        if ok:
            return True, last
        time.sleep(2.0)
    return False, last


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    add_pose_arguments(ap)
    ap.add_argument('--name', default='checkerboard')
    ap.add_argument('--no-pieces', action='store_true', help='the board alone')
    ap.add_argument('--no-camera', action='store_true', help='leave the camera of the Gazebo window where it is')
    args = ap.parse_args()
    board = (args.x, args.y, args.z, args.yaw)

    request = entity(model_file('checkerboard'), args.name, args.x, args.y, args.z, args.yaw)
    print(f'spawn_board: waiting for Gazebo world "{args.world}" ...', flush=True)
    deadline = time.monotonic() + args.timeout
    ok, last = retry(deadline, lambda: try_create(args.world, request))
    if not ok:
        sys.exit(f'spawn_board: gave up after {args.timeout:.0f} s. Last answer: {last or "(none)"}')
    print(f'spawn_board: "{args.name}" is in the world at '
          f'x={args.x:.3f} y={args.y:.3f} z={args.z:.3f} yaw={args.yaw:.3f}', flush=True)

    if not args.no_pieces:
        ok, last = retry(deadline, lambda: call(args.world, 'create_multiple', 'gz.msgs.EntityFactory_V',
                                                pieces_request(board)))
        if not ok:
            sys.exit(f'spawn_board: the board is in, but the pieces were not. Last answer: {last or "(none)"}')
        print(f'spawn_board: {len(start_position())} men on their starting squares', flush=True)

    if not args.no_camera:
        if move_camera(board, timeout=120.0):
            print('spawn_board: camera moved to look at the robot and the board', flush=True)
        else:
            print('spawn_board: no Gazebo window to move the camera of (headless?), left alone', flush=True)


if __name__ == '__main__':
    main()
