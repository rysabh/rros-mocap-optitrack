"""
Compatibility shim.

The ATI NetFT socket implementation lives in `ati_sensor_service` so the sensor
driver is defined in one place. We keep this module to avoid breaking imports in
older scripts under `data_collection`.
"""

from ati_sensor_service.submodules.ati_sensor_socket import (  # noqa: F401
    NetFTRDTPacket,
    NetFTSensor,
)

