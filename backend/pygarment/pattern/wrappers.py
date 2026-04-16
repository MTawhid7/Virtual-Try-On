"""
    Vendored & stripped wrappers from GarmentCode.

    Original uses cairosvg, svgwrite, and matplotlib for pattern visualization.
    This stripped version keeps the class hierarchy (VisPattern → BoxMesh)
    intact but replaces visualization methods with no-ops, eliminating the
    cairosvg/svgwrite dependency.

    Original: pygarment/pattern/wrappers.py (GarmentCode)
"""
from copy import copy
import random
import string
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

import svgpathtools as svgpath

# my
from pygarment import data_config
from . import core
from .utils import *


class VisPattern(core.ParametrizedPattern):
    """
    Pattern wrapper that extends ParametrizedPattern with visualization
    capabilities.  In this vendored version, the visualization methods are
    stubbed out so that cairosvg is not required.

    The serialize() method still writes the JSON spec (via the parent class),
    but skips SVG/PNG/PDF generation.
    """

    # ------------ Interface -------------

    def __init__(self, pattern_file=None):
        super().__init__(pattern_file)
        self.px_per_unit = 3

    def serialize(
            self, path, to_subfolder=True, tag='',
            with_3d=True, with_text=True, view_ids=True,
            with_printable=False,
            empty_ok=False,
            print_panel_dist=10,
        ):
        """Serialize pattern to disk.  Skips SVG/PNG visualization."""
        log_dir = super().serialize(path, to_subfolder, tag=tag, empty_ok=empty_ok)
        # NOTE: SVG/PNG/PDF generation removed — no cairosvg dependency
        return log_dir

    # -------- Drawing (stubbed) ---------

    def _verts_to_px_coords(self, vertices, translation_2d):
        """Convert given vertices and panel (2D) translation to px coordinate frame & units"""
        vertices[:, 1] *= -1
        translation_2d[1] *= -1
        offset = np.min(vertices, axis=0)
        vertices = vertices - offset
        translation_2d = translation_2d + offset
        return vertices, translation_2d

    def _flip_y(self, point):
        """To get to image coordinates one might need to flip Y axis"""
        flipped_point = list(point)
        flipped_point[1] *= -1
        return flipped_point

    def _draw_a_panel(self, panel_name, apply_transform=True, fill=True):
        """Draws a panel as an SVG path.  Kept for compatibility with BoxMesh
        which may call this indirectly."""
        attributes = {
            'fill': 'rgb(227,175,186)' if fill else 'rgb(255,255,255)',
            'stroke': 'rgb(51,51,51)',
            'stroke-width': '0.2'
        }

        panel = self.pattern['panels'][panel_name]
        vertices = np.asarray(panel['vertices'])
        vertices, translation = self._verts_to_px_coords(
            vertices,
            np.array(panel['translation'][:2]))

        segs = [self._edge_as_curve(vertices, edge) for edge in panel['edges']]
        path = svgpath.Path(*segs)
        if apply_transform:
            rotation = R.from_euler('XYZ', panel['rotation'], degrees=True)
            res = rotation.apply([0, 1, 0])
            flat_rot_angle = np.rad2deg(vector_angle([0, 1], res[:2]))
            path = path.rotated(
                degs=-flat_rot_angle,
                origin=list_to_c(vertices[0])
            )
            path = path.translated(list_to_c(translation))

        return path, attributes, panel['translation'][-1] >= 0

    def _save_as_image(self, svg_filename, png_filename,
                       with_text=True, view_ids=True, margin=2):
        """Stubbed — no cairosvg dependency."""
        pass

    def _save_as_image_3D(self, png_filename):
        """Stubbed — no matplotlib 3D dependency."""
        pass

    def _save_as_pdf(self, svg_filename, pdf_filename,
                     with_text=True, view_ids=True, margin=2):
        """Stubbed — no cairosvg dependency."""
        pass


class RandomPattern(VisPattern):
    """Parameter randomization of a pattern template."""

    def __init__(self, template_file):
        super().__init__(template_file, view_ids=False)
        self.name = self.name + '_' + self._id_generator()
        self._randomize_pattern()

    def _id_generator(self, size=10,
                      chars=string.ascii_uppercase + string.digits):
        return ''.join(random.choices(chars, k=size))
