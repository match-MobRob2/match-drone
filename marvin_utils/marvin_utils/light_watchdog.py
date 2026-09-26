#!/usr/bin/env python3
"""
LED State Node
Abhört /mavros/state und schaltet LEDs entsprechend (Priorität von oben nach unten):

  1. system_status == 8                        → alle LEDs schnell rot blinkend
  2. armed == False, mode != OFFBOARD           → alle LEDs gelb konstant
  3. armed == False, mode == OFFBOARD           → alle LEDs blau konstant
  4. armed == True,  mode != OFFBOARD           → Flugzeug-Modus:
                                                   vorne links  (3) = rot   konstant
                                                   vorne rechts (2) = grün  konstant
                                                   hinten (0+1)     = weiß  blinkend 1 s
  5. armed == True,  mode == OFFBOARD           → alle LEDs blau blinkend 1 s

LED-Anordnung:
  0 = hinten rechts
  1 = hinten links
  2 = vorne rechts
  3 = vorne links
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from mavros_msgs.msg import State
from marvin_msgs.srv import SetAllLeds, SetSingleLed


class LedStateNode(Node):

    # Blink-Intervalle in Sekunden
    BLINK_FAST_INTERVAL = 0.15   # system_status == 8: schnelles rotes Blinken
    BLINK_SLOW_INTERVAL = 1.0    # Flugzeug-Modus / OFFBOARD armed: 1-s-Blinken

    # Interne Bezeichner für den aktiven Blink-Modus
    _MODE_NONE      = 0
    _MODE_RED_FAST  = 1
    _MODE_AIRPLANE  = 2
    _MODE_BLUE_SLOW = 3

    def __init__(self):
        super().__init__('led_state_node')

        self.callback_group = ReentrantCallbackGroup()

        # Subscriber
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10,
            callback_group=self.callback_group,
        )

        # Service Clients
        self.set_all_client = self.create_client(
            SetAllLeds,
            '/set_all_leds',
            callback_group=self.callback_group,
        )
        self.set_single_client = self.create_client(
            SetSingleLed,
            '/set_single_led',
            callback_group=self.callback_group,
        )

        # Interner Zustand
        self._last_state: State | None = None
        self._blink_state = False          # Toggle für Blinken
        self._blink_timer = None           # Aktiver Blink-Timer
        self._active_mode = self._MODE_NONE

        # Warten auf Services (nicht blockierend, nur Warnung)
        for client, name in [
            (self.set_all_client, '/set_all_leds'),
            (self.set_single_client, '/set_single_led'),
        ]:
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f'Service {name} nicht verfügbar – warte weiterhin...')

        self.get_logger().info('LED State Node gestartet')

    # ------------------------------------------------------------------
    # Callback: neuer /mavros/state
    # ------------------------------------------------------------------
    def state_callback(self, msg: State):
        self._last_state = msg
        self._update_leds(msg)

    # ------------------------------------------------------------------
    # Haupt-Logik: LED-Modus bestimmen
    # ------------------------------------------------------------------
    def _update_leds(self, msg: State):
        """Entscheidet anhand des aktuellen States, welcher LED-Modus gilt."""

        # Priorität 1: system_status == 8 → schnelles rotes Blinken
        if msg.system_status == 8:
            self._enter_mode_red_fast()
            return

        offboard = (msg.mode == 'OFFBOARD')

        # Priorität 2: nicht armed, nicht OFFBOARD → gelb konstant
        if not msg.armed and not offboard:
            self._stop_blink()
            self._active_mode = self._MODE_NONE
            self._set_all(255, 200, 0)
            return

        # Priorität 3: nicht armed, OFFBOARD → blau konstant
        if not msg.armed and offboard:
            self._stop_blink()
            self._active_mode = self._MODE_NONE
            self._set_all(0, 0, 255)
            return

        # Ab hier: armed == True
        if not offboard:
            # Priorität 4: armed, kein OFFBOARD → Flugzeug-Modus
            self._enter_mode_airplane()
        else:
            # Priorität 5: armed, OFFBOARD → blau blinkend
            self._enter_mode_blue_slow()

    # ------------------------------------------------------------------
    # Modus-Einsprünge  (je nur aktiv wenn Modus sich wirklich ändert)
    # ------------------------------------------------------------------
    def _enter_mode_red_fast(self):
        if self._active_mode == self._MODE_RED_FAST:
            return
        self._stop_blink()
        self._active_mode = self._MODE_RED_FAST
        # Sofort Rot setzen, dann Timer
        self._set_all(255, 0, 0)
        self._blink_state = True
        self._blink_timer = self.create_timer(
            self.BLINK_FAST_INTERVAL, self._blink_red,
            callback_group=self.callback_group)

    def _enter_mode_airplane(self):
        if self._active_mode == self._MODE_AIRPLANE:
            return
        self._stop_blink()
        self._active_mode = self._MODE_AIRPLANE
        # Statische Farben vorne sofort setzen
        self._set_single(3, 255, 0, 0)    # vorne links   → rot
        self._set_single(2, 0, 255, 0)    # vorne rechts  → grün
        # Hinten sofort Weiß (erster Blink-Zustand = AN)
        self._set_single(0, 255, 255, 255)
        self._set_single(1, 255, 255, 255)
        self._blink_state = True           # nächster Tick schaltet AUS
        self._blink_timer = self.create_timer(
            self.BLINK_SLOW_INTERVAL, self._blink_white_airplane,
            callback_group=self.callback_group)

    def _enter_mode_blue_slow(self):
        if self._active_mode == self._MODE_BLUE_SLOW:
            return
        self._stop_blink()
        self._active_mode = self._MODE_BLUE_SLOW
        # Sofort Blau setzen (erster Blink-Zustand = AN)
        self._set_all(0, 0, 255)
        self._blink_state = True           # nächster Tick schaltet AUS
        self._blink_timer = self.create_timer(
            self.BLINK_SLOW_INTERVAL, self._blink_blue,
            callback_group=self.callback_group)

    # ------------------------------------------------------------------
    # Flugzeug-Modus Blink-Callback
    # ------------------------------------------------------------------
    def _blink_white_airplane(self):
        """Hintere LEDs (0+1) zwischen Aus und Weiß wechseln.
        _blink_state == True bedeutet: LEDs sind gerade AN → jetzt AUS schalten."""
        self._blink_state = not self._blink_state
        color = (255, 255, 255) if self._blink_state else (0, 0, 0)
        self._set_single(0, *color)   # hinten rechts
        self._set_single(1, *color)   # hinten links

    # ------------------------------------------------------------------
    # Blink-Callbacks
    # ------------------------------------------------------------------
    def _blink_red(self):
        """Alle LEDs schnell rot blinken. _blink_state == True → gerade AN."""
        self._blink_state = not self._blink_state
        if self._blink_state:
            self._set_all(255, 0, 0)
        else:
            self._set_all(0, 0, 0)

    def _blink_blue(self):
        """Alle LEDs blau blinken (1 s). _blink_state == True → gerade AN."""
        self._blink_state = not self._blink_state
        if self._blink_state:
            self._set_all(0, 0, 255)
        else:
            self._set_all(0, 0, 0)

    def _stop_blink(self):
        """Aktiven Blink-Timer stoppen und verwerfen."""
        if self._blink_timer is not None:
            self._blink_timer.cancel()
            self.destroy_timer(self._blink_timer)
            self._blink_timer = None
            self._blink_state = False

    # ------------------------------------------------------------------
    # Service-Aufrufe
    # ------------------------------------------------------------------
    def _set_all(self, r: int, g: int, b: int):
        if not self.set_all_client.service_is_ready():
            self.get_logger().warn('set_all_leds nicht verfügbar')
            return
        req = SetAllLeds.Request()
        req.r, req.g, req.b = r, g, b
        self.set_all_client.call_async(req)

    def _set_single(self, index: int, r: int, g: int, b: int):
        if not self.set_single_client.service_is_ready():
            self.get_logger().warn('set_single_led nicht verfügbar')
            return
        req = SetSingleLed.Request()
        req.index, req.r, req.g, req.b = index, r, g, b
        self.set_single_client.call_async(req)


# ------------------------------------------------------------------
# Entry Point
# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LedStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()