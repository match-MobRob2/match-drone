#!/usr/bin/env python3
"""End-to-End-Test des Supervisors ueber dieselbe Schnittstelle wie die Web-UI
(von Hand, Sim + bringup muessen laufen, nicht CI).

    python3 mission_e2e.py [karte]

Startet "Auf Karte fliegen", wartet auf gruene Ampel, speichert und startet eine
Mission, pausiert nach Wegpunkt 2 fuer 5 s, bestaetigt den Warte-Wegpunkt 3 ("Weiter")
und protokolliert alle Statuswechsel. Wegpunkt 2 hat feste Blickrichtung (yaw 90°).
"""
import json
import sys
import time

import rclpy
from marvin_msgs.srv import Command
from rclpy.node import Node
from std_msgs.msg import String

MAP = sys.argv[1] if len(sys.argv) > 1 else 'scale3'
WAYPOINTS = [{'x': 4.0, 'y': 0.0, 'z': 1.5, 'hold': 0},
             {'x': 4.0, 'y': 4.0, 'z': 1.5, 'hold': 3, 'yaw_deg': 90},
             {'x': -3.0, 'y': 4.0, 'z': 2.0, 'hold': 0, 'confirm': True},
             {'x': 0.0, 'y': 0.0, 'z': 1.5, 'hold': 0}]


class E2E(Node):
    def __init__(self):
        super().__init__('mission_e2e')
        self.cli = self.create_client(Command, '/marvin/command')
        self.status = None
        self.create_subscription(String, '/marvin/status', lambda m: setattr(self, 'status', json.loads(m.data)), 10)

    def cmd(self, **req):
        self.cli.wait_for_service()
        fut = self.cli.call_async(Command.Request(request=json.dumps(req)))
        rclpy.spin_until_future_complete(self, fut)
        r = fut.result()
        print(f'  {req["cmd"]}: {"ok" if r.success else "FEHLER"} {r.response[:120]}')
        return r.success

    def wait(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.status and cond(self.status):
                return True
        return False


def main():
    rclpy.init()
    n = E2E()
    t0 = time.time()
    n.cmd(cmd='start_localization', map=MAP)
    if not n.wait(lambda s: s['ready'], 240):
        print('nicht flugbereit:', [c for c in n.status['checks'] if not c['ok']])
        return
    print(f'[{time.time() - t0:5.0f}s] flugbereit')
    n.cmd(cmd='save_mission', map=MAP, name='e2e_test', waypoints=WAYPOINTS, land_at_end=True)
    n.cmd(cmd='start_mission', name='e2e_test')
    last, paused = None, False
    end = time.time() + 600
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        m = (n.status or {}).get('mission')
        if not m:
            continue
        key = (m['state'], m['index'], m['text'])
        if key != last:
            print(f'[{time.time() - t0:5.0f}s] {m["state"]:8} {m["index"] + 1}/{m["total"]}  {m["text"]}')
            last = key
        if not paused and m['state'] == 'running' and m['index'] == 2:
            n.cmd(cmd='pause_mission')
            n.wait(lambda s: s['mission']['state'] == 'paused', 30)
            time.sleep(5)
            n.cmd(cmd='resume_mission')
            paused = True
        if m['state'] == 'waiting':
            time.sleep(3)
            n.cmd(cmd='continue_mission')
            n.wait(lambda s: s['mission']['state'] != 'waiting', 10)
        if m['state'] in ('done', 'failed', 'aborted'):
            break
    print(f'Ergebnis: {m["state"]} — {m["text"]}')


if __name__ == '__main__':
    main()
