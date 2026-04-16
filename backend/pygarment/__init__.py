"""
    Vendored subset of pygarment (GarmentCode) — parametric sewing pattern library.
    
    Only the pattern DSL (garmentcode/), serialization (pattern/), and mesh
    generation (meshgen/) modules are included.  Simulation, rendering, GUI,
    and Maya modules are intentionally excluded.
    
    Original: https://github.com/maria-korosteleva/GarmentCode
    License: MIT
"""

# Building blocks
from pygarment.garmentcode.component import Component
from pygarment.garmentcode.panel import Panel
from pygarment.garmentcode.edge import Edge, CircleEdge, CurveEdge, EdgeSequence
from pygarment.garmentcode.connector import Stitches
from pygarment.garmentcode.interface import Interface
from pygarment.garmentcode.edge_factory import EdgeSeqFactory
from pygarment.garmentcode.edge_factory import CircleEdgeFactory
from pygarment.garmentcode.edge_factory import EdgeFactory
from pygarment.garmentcode.edge_factory import CurveEdgeFactory

# Operations
import pygarment.garmentcode.operators as ops
import pygarment.garmentcode.utils as utils

# Parameter support
from pygarment.garmentcode.params import BodyParametrizationBase, DesignSampler

# Errors
from pygarment.pattern.core import EmptyPatternError
