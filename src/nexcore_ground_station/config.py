"""Configuration management for the ground station application.

Provides JSON-based configuration loading and saving with sensible defaults.
Configuration is merged from a user-provided file over the built-in defaults.
"""

import json
import os
from pathlib import Path
from typing import Any, Optional

DEFAULT_CONFIG: dict[str, Any] = {
    "serial": {
        "baud": 115200,
        "timeout": 0.005,
        "auto_connect": False,
    },
    "wifi": {
        "port": 23,
        "timeout": 0.01,
    },
    "display": {
        "theme": "dark",
        "update_interval_ms": 50,
    },
    "logging": {
        "level": "INFO",
        "file": "",
    },
}


def load_config(path: Optional[str] = None) -> dict[str, Any]:
    """Load configuration from a JSON file, merged with defaults.

    If no path is given, checks the GS_CONFIG environment variable,
    then falls back to 'config.json' in the current directory.

    Args:
        path: Optional path to the JSON config file.

    Returns:
        A dict containing the merged configuration.
    """
    if path is None:
        path = os.environ.get("GS_CONFIG", "config.json")
    config_path = Path(path)
    if config_path.exists():
        with open(config_path) as f:
            user_config = json.load(f)
        merged = DEFAULT_CONFIG.copy()
        _deep_merge(merged, user_config)
        return merged
    return DEFAULT_CONFIG.copy()


def save_config(config: dict[str, Any], path: str = "config.json") -> None:
    """Save configuration to a JSON file.

    Args:
        config: Configuration dict to save.
        path: File path to write to.
    """
    with open(path, "w") as f:
        json.dump(config, f, indent=2)


def _deep_merge(base: dict, override: dict) -> None:
    """Recursively merge override dict into base dict.

    Args:
        base: The target dict to merge into.
        override: The source dict with override values.
    """
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
