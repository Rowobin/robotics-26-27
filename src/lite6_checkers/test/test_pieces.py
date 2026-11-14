"""The pieces are the ones of the challenge (base D 24 x 10 mm, peg D 11 x 15 mm) and start where the
rules put them."""
import pathlib
import xml.etree.ElementTree as ET

import pytest

from lite6_checkers import make_piece_sdf as mp
from lite6_checkers import spawn_board as sb

MODELS = pathlib.Path(__file__).resolve().parents[1] / 'models'


def test_dimensions_are_the_challenges():
    assert mp.BASE_D == pytest.approx(0.024)
    assert mp.BASE_H == pytest.approx(0.010)
    assert mp.PEG_D == pytest.approx(0.011)
    assert mp.PEG_H == pytest.approx(0.015)
    assert mp.PEG_D < 0.016                 # the gripper opens to 16 mm


def test_the_peg_is_the_handle_a_gripper_can_close_on():
    root = ET.fromstring(mp.piece_sdf('white'))
    peg = [c for c in root.iter('collision') if c.get('name') == 'peg'][0]
    assert float(peg.find('geometry/cylinder/radius').text) * 2 == pytest.approx(0.011)
    assert float(peg.find('geometry/cylinder/length').text) == pytest.approx(0.015)


def test_the_centre_of_mass_is_in_the_peg_above_the_base_top():
    m, zc, Ixx, Izz = mp.mass_properties()
    assert 0.004 < m < 0.015                # a few grams
    assert mp.BASE_H * 0.5 < zc < mp.BASE_H + mp.PEG_H
    assert zc == pytest.approx(0.008, abs=0.001)
    assert Ixx > 0 and Izz > 0


def test_a_dama_has_the_shape_of_a_man_and_a_gold_cap():
    man = ET.fromstring(mp.piece_sdf('black', dama=False))
    dama = ET.fromstring(mp.piece_sdf('black', dama=True))
    assert [c.get('name') for c in man.iter('collision')] == [c.get('name') for c in dama.iter('collision')]
    assert 'cap' in [v.get('name') for v in dama.iter('visual')]
    assert 'cap' not in [v.get('name') for v in man.iter('visual')]


@pytest.mark.parametrize('name', ['checker_white', 'checker_black', 'dama_white', 'dama_black'])
def test_the_installed_models_are_what_the_generator_produces(name):
    sdf, config = mp.models()[name]
    assert (MODELS / name / 'model.sdf').read_text() == sdf, \
        'models/*/model.sdf is stale: python3 -m lite6_checkers.make_piece_sdf models/'
    assert (MODELS / name / 'model.config').read_text() == config


def test_starting_position_is_12_men_a_side_on_the_dark_squares_of_three_rows():
    start = sb.start_position()
    assert len(start) == 24
    assert sum(k == 'checker_white' for k in start.values()) == 12
    assert sum(k == 'checker_black' for k in start.values()) == 12
    for sq, kind in start.items():
        f, r = sb.parse_square(sq)
        assert (f + r) % 2 == 0, f'{sq} is a light square'
        assert (r <= 2) == (kind == 'checker_white')
        assert (r >= 5) == (kind == 'checker_black')
    assert start['a1'] == 'checker_white' and start['h8'] == 'checker_black'
    assert 'b1' not in start and 'a2' not in start            # light squares


def test_squares_land_where_the_board_is():
    board = (sb.DEFAULT_X, sb.DEFAULT_Y, sb.DEFAULT_Z, sb.DEFAULT_YAW)
    # a1: White's near-left corner. The robot faces -y, White's side is towards it, White's left is +x.
    assert sb.to_world(board, *sb.parse_square('a1')) == pytest.approx((-0.095, -0.635))
    assert sb.to_world(board, *sb.parse_square('h8')) == pytest.approx((-0.305, -0.845))


def test_parse_square_rejects_nonsense():
    with pytest.raises(ValueError):
        sb.parse_square('i9')
    assert sb.parse_square('D4') == (3, 3)


def test_the_camera_looks_at_the_scene_from_nearby():
    import math
    board = (sb.DEFAULT_X, sb.DEFAULT_Y, sb.DEFAULT_Z, sb.DEFAULT_YAW)
    px, py, pz, qx, qy, qz, qw = sb.camera_pose(board)
    assert math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) == pytest.approx(1.0)
    # the camera looks along its own +x: rotate (1, 0, 0) by the quaternion
    fx = 1 - 2 * (qy * qy + qz * qz)
    fy = 2 * (qx * qy + qw * qz)
    fz = 2 * (qx * qz - qw * qy)
    tx = board[0] + sb.CAMERA_TARGET_SHIFT[0]
    ty = board[1] + sb.CAMERA_TARGET_SHIFT[1]
    tz = board[2] + sb.CAMERA_TARGET_SHIFT[2]
    d = math.dist((px, py, pz), (tx, ty, tz))
    assert (px + fx * d, py + fy * d, pz + fz * d) == pytest.approx((tx, ty, tz), abs=1e-6)
    assert 0.5 < d < 1.2                    # close, not the 3 m of Gazebo's default view
    assert pz > tz                          # from above
