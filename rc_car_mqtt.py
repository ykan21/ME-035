# rc_car_mqtt.py -- MicroPython (Thonny) on ESP32 + Cytron ROBO-ESP32 -- Homework 9
# The robot subscribes to MQTT topics and does what the broker tells it:
#   ME35/speed      integer 0-100      -> sets/changes speed only (never starts the motors by itself)
#   ME35/direction  forward|backward|left|right|stop
#   ME35/led/0      {"r":255,"g":0,"b":122}   NeoPixel 0
#   ME35/led/1      {"r":255,"g":0,"b":122}   NeoPixel 1
# Lost connection (WiFi or broker) -> motors STOP and the lights blink RED until it reconnects.

from machine import Pin, PWM
import neopixel
import network
import machine
import ubinascii
import json
import time
from umqtt.simple import MQTTClient

# ======================= CONFIG =======================
WIFI_SSID = "tufts_eecs"
WIFI_PASSWORD = "foundedin1883"

BROKER_IP = "BrokersIP" #change to the instructor's IP on demo day
BROKER_PORT = 1883

# Motors (same as the RC car / line follower)
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True
LEFT_INVERT  = False

NEOPIXEL_PIN = 15
NUM_PIXELS = 2

SWAP_TURNS = True                # True: "left"/"right" commands rotate the opposite way from before (fixes mirrored turns)
BACK_TRIM_L = 1.0                # scale each wheel while driving BACKWARD (1.0 = no change)
BACK_TRIM_R = 1.0                # if it curves toward the robot's right in reverse, lower BACK_TRIM_L (e.g. 0.85)

DEFAULT_SPEED = 20               # percent, used until a speed message arrives
MIN_DRIVE = 0.20                 # smallest duty that actually moves the car (any speed > 0 gets at least this)
PING_EVERY_MS = 2000             # MQTT keepalive ping interval (helps notice a dead link)
RECONNECT_EVERY_MS = 2000        # how often to retry while disconnected
BLINK_MS = 300

TOPIC_SPEED = b"ME35/speed"
TOPIC_DIR = b"ME35/direction"
TOPIC_LED = (b"ME35/led/0", b"ME35/led/1")
# ======================================================


# ---------- Motors ----------
class Motor:
    def __init__(self, pa, pb, invert):
        self.a = PWM(Pin(pa), freq=20000, duty_u16=0)
        self.b = PWM(Pin(pb), freq=20000, duty_u16=0)
        self.invert = invert

    def set(self, speed):
        speed = max(-1.0, min(1.0, speed))
        if self.invert:
            speed = -speed
        duty = int(abs(speed) * 65535)
        if speed >= 0:
            self.a.duty_u16(duty)
            self.b.duty_u16(0)
        else:
            self.a.duty_u16(0)
            self.b.duty_u16(duty)


right = Motor(RIGHT_A, RIGHT_B, RIGHT_INVERT)
left = Motor(LEFT_A, LEFT_B, LEFT_INVERT)


def drive(l, r):
    left.set(l)
    right.set(r)


def stop():
    drive(0, 0)


# ---------- NeoPixels ----------
np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)
led_colors = [(0, 0, 0)] * NUM_PIXELS      # what the instructor asked for (restored after a blink)


def show_leds():
    for i in range(NUM_PIXELS):
        np[i] = led_colors[i]
    np.write()


def show_red(on):
    for i in range(NUM_PIXELS):
        np[i] = (255, 0, 0) if on else (0, 0, 0)
    np.write()


def blink_red():
    """Call often while disconnected; blinks the pixels red without blocking."""
    show_red((time.ticks_ms() // BLINK_MS) % 2 == 0)


show_leds()


# ---------- Robot state ----------
speed = DEFAULT_SPEED           # 0-100
direction = "stop"


def duty_from_speed():
    if speed <= 0:
        return 0.0
    return max(MIN_DRIVE, min(1.0, speed / 100.0))


def apply_motion():
    """Turn the current (direction, speed) into motor commands."""
    d = duty_from_speed()
    if direction == "forward":
        drive(d, d)
    elif direction == "backward":
        drive(-d * BACK_TRIM_L, -d * BACK_TRIM_R)
    elif direction == "left":        # rotate left in place
        if SWAP_TURNS:
            drive(d, -d)
        else:
            drive(-d, d)
    elif direction == "right":       # rotate right in place
        if SWAP_TURNS:
            drive(-d, d)
        else:
            drive(d, -d)
    else:
        stop()


# ---------- MQTT message handling ----------
def on_message(topic, msg):
    global speed, direction
    try:
        text = msg.decode().strip()
        if topic == TOPIC_SPEED:
            speed = max(0, min(100, int(float(text))))
            print("speed ->", speed)
            if direction != "stop":          # a speed change updates a move in progress,
                apply_motion()               # but never starts one
        elif topic == TOPIC_DIR:
            text = text.lower()
            if text in ("forward", "backward", "left", "right", "stop"):
                direction = text
                print("direction ->", direction, "at speed", speed)
                apply_motion()
            else:
                print("unknown direction:", text)
        elif topic in TOPIC_LED:
            idx = TOPIC_LED.index(topic)
            c = json.loads(text)
            r = max(0, min(255, int(c.get("r", 0))))
            g = max(0, min(255, int(c.get("g", 0))))
            b = max(0, min(255, int(c.get("b", 0))))
            led_colors[idx] = (r, g, b)
            print("led", idx, "->", led_colors[idx])
            show_leds()
    except Exception as e:                   # a bad message must never crash the robot
        print("bad message on", topic, msg, e)


# ---------- WiFi ----------
wlan = network.WLAN(network.STA_IF)
wlan.active(True)


def wifi_ok():
    return wlan.isconnected()


def wifi_connect():
    """Blocks (blinking red) until WiFi is up."""
    if wifi_ok():
        return
    print("Connecting to WiFi...")
    try:
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    except OSError:
        pass
    last_try = time.ticks_ms()
    while not wifi_ok():
        blink_red()
        time.sleep_ms(50)
        if time.ticks_diff(time.ticks_ms(), last_try) > 10000:   # retry if it's taking too long
            try:
                wlan.disconnect()
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            except OSError:
                pass
            last_try = time.ticks_ms()
    print("WiFi connected, robot IP:", wlan.ifconfig()[0])


# ---------- MQTT connection ----------
CLIENT_ID = b"esp32_" + ubinascii.hexlify(machine.unique_id())   # unique per board (important with many bots)


def mqtt_connect():
    c = MQTTClient(CLIENT_ID, BROKER_IP, port=BROKER_PORT, keepalive=10)
    c.set_callback(on_message)
    try:
        c.connect(timeout=2)                 # newer umqtt.simple
    except TypeError:
        c.connect()                          # older umqtt.simple has no timeout argument
    c.subscribe(TOPIC_SPEED)
    c.subscribe(TOPIC_DIR)
    for t in TOPIC_LED:
        c.subscribe(t)
    return c


def connection_lost():
    stop()
    print("CONNECTION LOST - stopped, blinking red")


def session(c):
    """Runs while connected. Returns when the link is lost."""
    last_ping = time.ticks_ms()
    while True:
        if not wifi_ok():
            raise OSError("wifi down")
        c.check_msg()                        # raises OSError if the broker closed the connection
        if time.ticks_diff(time.ticks_ms(), last_ping) > PING_EVERY_MS:
            c.ping()                         # raises OSError if the socket is dead
            last_ping = time.ticks_ms()
        time.sleep_ms(10)


def main():
    global direction
    stop()
    while True:
        wifi_connect()
        c = None
        try:
            c = mqtt_connect()
            print("Connected to broker", BROKER_IP)
            show_leds()                      # back to the instructor's colors
            session(c)
        except Exception as e:               # OSError, MQTTException, anything: treat as lost link
            print("link error:", e)
        # ---- connection lost: STOP, blink RED, retry ----
        direction = "stop"
        connection_lost()
        try:
            if c:
                c.sock.close()
        except Exception:
            pass
        wait_until = time.ticks_add(time.ticks_ms(), RECONNECT_EVERY_MS)
        while time.ticks_diff(wait_until, time.ticks_ms()) > 0:
            blink_red()
            time.sleep_ms(50)


main()
