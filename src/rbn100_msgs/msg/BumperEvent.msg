# Provides a bumper event.
# This message is generated whenever a particular bumper is pressed or released.
# Note that, despite bumper field on SensorState messages, state field is not a
# bitmask, but the new state of a single sensor.

# bumper
uint8 body = 0

# state
uint8 RELEASED = 0
uint8 PRESSED  = 1

uint8 bumper
uint8 state

Header header
