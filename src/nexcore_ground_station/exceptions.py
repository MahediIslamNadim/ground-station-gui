class GroundStationError(Exception):
    """Base exception for all ground station errors."""


class SerialError(GroundStationError):
    """Raised when serial port operations fail."""


class MAVLinkError(GroundStationError):
    """Raised when MAVLink protocol parsing fails."""


class ConfigError(GroundStationError):
    """Raised when configuration loading or saving fails."""
