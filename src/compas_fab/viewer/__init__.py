"""
********************************************************************************
compas_fab.viewer
********************************************************************************

.. currentmodule:: compas_fab.viewer

Classes
-------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    TrajectoryPlayer
    WorkpieceManager

"""
from .trajectory_player import TrajectoryPlayer
# Assuming WorkpieceManager is defined in trajectory_player.py
# If it's in a different file, change the import accordingly, e.g., from .workpiece_manager import WorkpieceManager
from .workpiece_manager import WorkpieceManager 

__all__ = [
    "TrajectoryPlayer",
    "WorkpieceManager",
]