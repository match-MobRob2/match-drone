**teleop_twist_keyboard** takes keypresses from the keyboard and publishes them as Twist
messages. It works best with a US keyboard layout.
For original documentation check this link out: https://index.ros.org/r/teleop_twist_keyboard/


### 🕹️ Move Bindings
```python
moveBindings = {
   'i': (1, 0, 0, 0),               # forward
   'o': (1, 0, 0, -1),              # forward + yaw right
   'j': (0, 0, 0, 1),               # yaw left
   'l': (0, 0, 0, -1),              # yaw right
   'u': (1, 0, 0, 1),               # forward + yaw left
   ',': (-1, 0, 0, 0),              # backward
   '.': (-1, 0, 0, 1),              # backward + yaw left
   'm': (-1, 0, 0, -1),             # backward + yaw right
   'O': (1, -1, 0, 0),              # forward + right strafe
   'I': (1, 0, 0, 0),               # forward (same as 'i')
   'J': (0, 1, 0, 0),               # left strafe
   'L': (0, -1, 0, 0),              # right strafe
   'U': (1, 1, 0, 0),               # forward + left strafe
   '<': (-1, 0, 0, 0),              # backward (same as ',')
   '>': (-1, -1, 0, 0),             # backward + right strafe
   'M': (-1, 1, 0, 0),              # backward + left strafe
   't': (0, 0, 1, 0),               # up
   'b': (0, 0, -1, 0),              # down
}
```
### 🕹️ Speed Bindings

```python
speedBindings = {
    'q': (1.1, 1.1),                # increase both linear and angular speed
    'z': (0.9, 0.9),                # decrease both linear and angular speed
    'w': (1.1, 1.0),                # increase linear speed only
    'x': (0.9, 1.0),                # decrease linear speed only
    'e': (1.0, 1.1),                # increase angular speed only
    'c': (1.0, 0.9),                # decrease angular speed only
}
```