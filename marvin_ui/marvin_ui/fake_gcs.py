#!/usr/bin/env python3
"""Minimal-Bodenstation fuer Headless-SITL: sendet GCS-Heartbeats an PX4.

Das Airframe setzt NAV_DLL_ACT=2 -> ohne GCS-Link verweigert PX4 das Armen
('No connection to the ground control station'). Statt den Sicherheits-
Parameter zu verstellen (bleibt in parameters.bson haengen) ersetzt das hier
QGC. Lauscht wie QGC auf UDP 14550.
"""
import time

from pymavlink import mavutil

def main():
    m = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)
    print('warte auf PX4 ...', flush=True)
    m.wait_heartbeat()
    print('verbunden', flush=True)
    while True:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        while m.recv_match(blocking=False):
            pass
        time.sleep(1.0)


if __name__ == '__main__':
    main()
