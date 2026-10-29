"""
MicroPython code to communicate with the AquaPing sensor on a Raspberry Pi Pico.
Flash this firmware on the RP with an IDE such as Thonny. Re-name it main.py
so it launches automatically at bootup.

Periodically reads 12 status bytes from the AquaPing and prints them
on USB virtual serial. Also sends command bytes from the host PC.
Requires the companion program aquaping_gui.py on the PC.

Wiring (Pico is 3.3 V logic; do not apply 5 V to any pin):
  GP4 (pin 6)  SDA  <-> AquaPing SDA   (needs external pull-up, e.g. 4.7k to 3.3 V)
  GP5 (pin 7)  SCL  <-> AquaPing SCL   (needs external pull-up, e.g. 4.7k to 3.3 V)
  GP3 (pin 5)  LEAK_PIN  <- ALARM on sensor
  GP2 (pin 4)  NOISE_PIN <- NOISE on sensor
  GND common with sensor

GPIO23 drives the Pico's SMPS PS pin. HIGH forces PWM mode (lower 3.3 V ripple).
Plain Pico only; on a Pico W, GPIO23 is the wireless power pin and does not control the SMPS.

Licensed under Creative Commons
MicroPhonon October 2026
"""

import sys
import select
import time
from machine import Pin, I2C
import micropython

SDA_PIN = 4
SCL_PIN = 5
LEAK_PIN = 3
NOISE_PIN = 2
SMPS_PS_PIN = 23      # HIGH = PWM mode, lower ripple (plain Pico only)
SBYTES = 12           # Number of status bytes
ISTRING = 8           # Maximum command length in bytes

POLLING_INTERVAL = 1000   # ms; raise to decrease polling rate
SERIAL_POLL = 200         # ms; sets responsiveness of GUI
MIN_INTERVAL = 0          # Delay to escape polling when an alarm occurs; should be 0
CMD_GAP = 40              # A command ends after this many ms with no new input bytes
I2C_SLAVE = 0x77          # AquaPing slave address
I2C_FREQ = 100_000

# Commands may contain byte value 3 (Ctrl-C), which MicroPython would otherwise treat
# as KeyboardInterrupt. Disable that for the life of the program. To stop the script
# from Thonny, comment this line out or reset the board.
micropython.kbd_intr(-1)

# Reduce supply ripple first: force the on-board SMPS into PWM mode
smps = Pin(SMPS_PS_PIN, Pin.OUT, value=1)

led = Pin("LED", Pin.OUT, value=0)   # "LED" works on Pico and Pico W
leak_pin = Pin(LEAK_PIN, Pin.IN, Pin.PULL_DOWN)    # Pull-downs keep floating lines quiet
noise_pin = Pin(NOISE_PIN, Pin.IN, Pin.PULL_DOWN)

i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)

# Glitch flags set by the interrupt handlers (a flag is 1 only if the pin was still
# high at the last edge, so glitches are ignored)
leak_edge = 0
noise_edge = 0


def alarm_handler(pin):   # No allocation or printing in an IRQ handler
    global leak_edge
    leak_edge = 1 if pin.value() else 0


def noise_handler(pin):
    global noise_edge
    noise_edge = 1 if pin.value() else 0


leak_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=alarm_handler)
noise_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=noise_handler)

# Non-blocking check for USB serial input
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
stdin = sys.stdin.buffer


def read_command():
    """Read one command: bytes arrive until CMD_GAP ms pass with no new input.
    Returns (bytes, overflow_flag). No terminator is expected."""
    buf = bytearray()
    overflow = False
    last_rx = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), last_rx) < CMD_GAP:
        while poller.poll(0):
            b = stdin.read(1)
            if not b:
                break
            last_rx = time.ticks_ms()
            if len(buf) < ISTRING:
                buf.extend(b)
            else:
                overflow = True
        time.sleep_ms(1)
    return bytes(buf), overflow


def request_status():
    """Read SBYTES from the slave. Returns bytes, or None on communication failure."""
    try:
        return i2c.readfrom(I2C_SLAVE, SBYTES)
    except OSError:
        return None


previous_ms = time.ticks_ms()

while True:
    # Check for input from the control program on the serial port
    if poller.poll(0):
        cmd, overflow = read_command()
        if overflow:
            print("Command too long; ignored.")
        elif cmd:
            # Send byte string to slave on I2C exactly as received
            try:
                i2c.writeto(I2C_SLAVE, cmd)
            except OSError as e:
                print("I2C command error", e.errno)

    now = time.ticks_ms()
    # Check for leak or noise alert from interrupts; do not wait for the polling loop
    if leak_edge == 1 or noise_edge == 1:
        trigger = MIN_INTERVAL
    else:
        trigger = POLLING_INTERVAL
    leak_edge = 0
    noise_edge = 0

    # Retrieve data from sensor at the designated polling time
    if time.ticks_diff(now, previous_ms) >= trigger:
        led.on()
        previous_ms = now
        data = request_status()
        if data is None:   # Unusual, but allow the slave to reset just in case
            print("No bytes available on I2C. Retry...")
            time.sleep_ms(700)   # Wait for I2C bus to reset
            data = request_status()
            if data is None:
                print("Retry failed.")   # Communication problem; check I2C configuration
            else:
                print("Retry successful.")

        if data is not None:
            # Convert to decimal and send to host (trailing space matches the Arduino output)
            print("".join("%d " % b for b in data))
        led.off()

    time.sleep_ms(SERIAL_POLL)   # Needed to read serial port
