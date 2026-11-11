"""
Put one piece on a square of the checkerboard in the running Gazebo world.

    ros2 run lite6_checkers spawn_piece dama_white d4
    ros2 run lite6_checkers spawn_piece checker_black e5 --name black_extra

KIND is checker_white, checker_black, dama_white or dama_black; the square is a1 .. h8 (a1 is the
dark corner at White's left). The board is assumed where `spawn_board` puts it; pass --x, --y, --z
and --yaw if you moved it. Use it to try a promotion (a dama in place of a man) or to put a piece
back after it was knocked off.
"""
import argparse
import sys
import time

from lite6_checkers import spawn_board as sb


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('kind', choices=sb.KINDS)
    ap.add_argument('square')
    ap.add_argument('--name', help='the name of the entity (default: <colour>_<square>, or dama_<colour>_<square>)')
    sb.add_pose_arguments(ap)
    args = ap.parse_args()
    try:
        sb.parse_square(args.square)
    except ValueError as e:
        sys.exit(f'spawn_piece: {e}')
    board = (args.x, args.y, args.z, args.yaw)
    name = args.name or sb.instance_name(args.kind, args.square)
    request = sb.piece_entity(board, args.kind, args.square, name=name)
    ok, last = sb.retry(time.monotonic() + args.timeout, lambda: sb.try_create(args.world, request))
    if not ok:
        sys.exit(f'spawn_piece: not created. Last answer: {last or "(none)"}')
    print(f'spawn_piece: {args.kind} "{name}" on {args.square}', flush=True)


if __name__ == '__main__':
    main()
