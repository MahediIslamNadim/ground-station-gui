import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)

logger = logging.getLogger("nexcore_gs")

from .app import GroundStation, SerialConn, MAVLink, Theme, ScrollFrame
from .app import main
from .config import load_config, save_config

__all__ = [
    "GroundStation", "SerialConn", "MAVLink", "Theme", "ScrollFrame",
    "main", "logger", "load_config", "save_config",
]
