# Motor Stall state published at a fixed frequency by the driver

Header header

# XXX: wheel indices in their respective arrays does not necessarily
# reflect what wheel it actually represents
# Each of these messages has its own internal field for that purpose.
# Example: check motor[0].motor == Motor.RIGHTWHEEL and do not rely on the 0

Motor[5] motor
