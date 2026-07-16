"""ADS1115 CP voltage reader for the Nextion status text."""

from __future__ import annotations

import site as _site
import sys
import threading
from dataclasses import dataclass
from typing import Optional

try:
    user_site = _site.getusersitepackages()
    if user_site and user_site not in sys.path:
        _site.addsitedir(user_site)
except Exception:
    pass


@dataclass(frozen=True)
class CpReading:
    adc_voltage: float
    cp_voltage: float
    status: Optional[str]


def status_from_cp_voltage(cp_voltage: float) -> Optional[str]:
    if cp_voltage >= 11.1:
        return "Available"
    if 7.0 <= cp_voltage <= 11.0:
        return "Connected"
    if 5.0 <= cp_voltage <= 7.0:
        return "Charging"
    if 2.0 <= cp_voltage <= 4.0:
        return "Ventilation"
    if cp_voltage < 1.0:
        return "Fault"
    return None


class CpSignalReader:
    def __init__(self, channel: int = 0, gain: int = 1, voltage_multiplier: float = 4.0303):
        self.channel = channel
        self.gain = gain
        self.voltage_multiplier = voltage_multiplier
        self._ads = None
        self._chan = None
        self._lock = threading.Lock()

    def connect(self):
        if self._chan is not None:
            return

        import board
        import busio
        import adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15.analog_in import AnalogIn

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        ads.gain = self.gain

        ads_pin = getattr(ADS, f"P{self.channel}", self.channel)
        self._ads = ads
        self._chan = AnalogIn(ads, ads_pin)

    def read(self) -> CpReading:
        with self._lock:
            self.connect()
            adc_voltage = float(self._chan.voltage)

        cp_voltage = adc_voltage * self.voltage_multiplier
        return CpReading(
            adc_voltage=adc_voltage,
            cp_voltage=cp_voltage,
            status=status_from_cp_voltage(cp_voltage),
        )
