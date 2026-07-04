class GroundStationError(Exception):
    pass


class SerialError(GroundStationError):
    pass


class MAVLinkError(GroundStationError):
    pass


class ConfigError(GroundStationError):
    pass
