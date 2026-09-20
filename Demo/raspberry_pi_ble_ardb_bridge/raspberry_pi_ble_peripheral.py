#!/usr/bin/env python3
"""Expose a Raspberry Pi's bytes as BLE notifications for the ESP32 bridge.

The script reads one newline-delimited message from standard input at a time.
Each line (without its trailing newline) becomes one ARDB Binary payload. A
line must be at most 64 bytes. Sensor code can instead call Bridge.send() with
the bytes it wants to publish.

Requires BlueZ and Bluezero. Run this process with the privileges required to
register a local GATT application, typically via sudo on Raspberry Pi OS.
"""

from __future__ import annotations

import sys
import threading

from gi.repository import GLib
from bluezero import adapter, peripheral


SERVICE_UUID = "e7e1b8a5-7d13-44e7-9e85-7a5638d8a100"
DATA_CHARACTERISTIC_UUID = "e7e1b8a5-7d13-44e7-9e85-7a5638d8a101"
LOCAL_NAME = "ARDB Pi Bridge"

PROTOCOL_VERSION = 0xA1
FRAME_START = 0x01
FRAME_END = 0x02
FRAME_HEADER_BYTES = 4
MAX_APPLICATION_BYTES = 64
# A 20-byte notification works with the default BLE ATT MTU of 23 bytes.
MAX_NOTIFICATION_BYTES = 20
MAX_FRAGMENT_BYTES = MAX_NOTIFICATION_BYTES - FRAME_HEADER_BYTES


class Bridge:
    def __init__(self) -> None:
        self._characteristic = None
        self._sequence = 0

    def on_notify(self, notifying, characteristic) -> None:
        self._characteristic = characteristic if notifying else None
        print("ESP32 subscribed" if notifying else "ESP32 unsubscribed", flush=True)

    def send(self, payload: bytes) -> bool:
        """Send one complete application message; called on GLib's thread."""
        if self._characteristic is None:
            print("drop: ESP32 has not subscribed", file=sys.stderr, flush=True)
            return False
        if len(payload) > MAX_APPLICATION_BYTES:
            print(f"drop: {len(payload)} bytes exceeds {MAX_APPLICATION_BYTES}",
                  file=sys.stderr, flush=True)
            return False

        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        offset = 0
        first = True
        while first or offset < len(payload):
            fragment = payload[offset:offset + MAX_FRAGMENT_BYTES]
            offset += len(fragment)
            flags = FRAME_START if first else 0
            if offset == len(payload):
                flags |= FRAME_END
            packet = bytes((PROTOCOL_VERSION, sequence, flags, len(payload))) + fragment
            self._characteristic.set_value(list(packet))
            first = False
        return False  # GLib.idle_add: do not schedule this callback again.


def read_stdin(bridge: Bridge) -> None:
    for line in sys.stdin.buffer:
        # Newlines delimit messages; all other bytes are passed through unchanged.
        payload = line.rstrip(b"\r\n")
        GLib.idle_add(bridge.send, payload)


def main() -> None:
    available_adapters = list(adapter.Adapter.available())
    if not available_adapters:
        raise RuntimeError("No Bluetooth adapter found")

    bridge = Bridge()
    gatt = peripheral.Peripheral(available_adapters[0].address,
                                 local_name=LOCAL_NAME)
    gatt.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)
    gatt.add_characteristic(
        srv_id=1,
        chr_id=1,
        uuid=DATA_CHARACTERISTIC_UUID,
        value=[],
        notifying=False,
        flags=["notify"],
        notify_callback=bridge.on_notify,
        read_callback=None,
        write_callback=None,
    )

    threading.Thread(target=read_stdin, args=(bridge,), daemon=True).start()
    print(f"Advertising {LOCAL_NAME}; write up to {MAX_APPLICATION_BYTES} bytes per line to stdin",
          flush=True)
    gatt.publish()


if __name__ == "__main__":
    main()
