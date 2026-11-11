"""The checkerboard is the one of the challenge: 8 x 8 squares of 30 mm, a1 dark."""
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest

from lite6_checkers import make_board_sdf as mb

MODEL = pathlib.Path(__file__).resolve().parents[1] / 'models' / 'checkerboard' / 'model.sdf'


def test_playing_area_is_240_mm():
    assert mb.SQUARES == 8
    assert mb.SQUARE == pytest.approx(0.030)
    assert mb.SQUARES * mb.SQUARE == pytest.approx(0.240)


def test_the_installed_model_is_what_the_generator_produces():
    assert MODEL.read_text() == mb.board_sdf(), \
        'models/checkerboard/model.sdf is stale: python3 -m lite6_checkers.make_board_sdf > it'


def test_32_dark_squares_and_a_light_plate_of_the_playing_area():
    root = ET.fromstring(mb.board_sdf())
    visuals = {v.get('name'): v for v in root.iter('visual')}
    dark = [n for n in visuals if n.startswith('dark_')]
    assert len(dark) == 32
    size = [float(s) for s in visuals['light_plate'].find('geometry/box/size').text.split()]
    assert size[:2] == pytest.approx([0.240, 0.240])


def test_a1_is_dark_and_the_long_diagonal_is_dark():
    assert mb.is_dark(0, 0)                                  # a1
    assert all(mb.is_dark(i, i) for i in range(8))           # a1 ... h8
    assert not mb.is_dark(7, 0)                              # h1 is light


def test_squares_sit_on_a_30_mm_grid_centred_on_the_origin():
    x_a1, y_a1 = mb.square_centre(0, 0)
    x_h8, y_h8 = mb.square_centre(7, 7)
    # a1 is White's near-left corner: x towards Black is negative, y towards the a-file positive
    assert (x_a1, y_a1) == pytest.approx((-0.105, 0.105))
    assert (x_h8, y_h8) == pytest.approx((0.105, -0.105))
    assert mb.square_centre(0, 1)[0] - x_a1 == pytest.approx(0.030)


def test_collision_surface_is_flat_with_its_top_on_the_playing_plane():
    root = ET.fromstring(mb.board_sdf())
    pose = [float(s) for s in root.find('.//collision/pose').text.split()]
    size = [float(s) for s in root.find('.//collision/geometry/box/size').text.split()]
    assert pose[2] + size[2] / 2 == pytest.approx(0.0)
