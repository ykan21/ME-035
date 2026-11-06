# controller_ble.py -- MicroPython (Thonny) on ESP32 + Cytron ROBO-ESP32  -- the CONTROLLER
# BLE role: Listener (central). It finds the car (Yeller) by NAME and connects.
# Joystick -> drive commands, HONK button, BOOST button (+ 2 LEDs).
# When the car reports the finish line, the controller ACKs and celebrates.
# Needs BLE_CEEO.py on the board (the class library).

from BLE_CEEO import Listen
import time
import neopixel
from machine import Pin, ADC, PWM

# "run"  -> be the controller
# "test" -> print joystick + button readings (use this FIRST to check wiring / pins)
MODE = "run"

# ======================= CONFIG =======================
NAME = "team1car"                # BLE name: must match rc_car_ble.py and be unique in the class

# Joystick: the two analog outputs of the Grove port (use ADC-capable pins).
JOY_X_PIN = 32                   # joystick X (D32)
JOY_Y_PIN = 39                   # joystick Y (D39)
JOY_X_INVERT = True              # D32 reads HIGHER when pushed left, so flip it
JOY_Y_INVERT = False             # flip if pushing UP makes the car go BACKWARD
DEADZONE = 0.12                  # ignore small wobble around center
THROW = 0.90                     # joystick usually only reaches ~90% of the ADC range
RAW_MIN, RAW_MAX = 0, 65535      # ADC range (read_u16); tune with MODE = "test"

# Buttons: one leg to the GPIO, other leg to GND (internal pull-up, pressed = LOW).
# Don't use GPIO 34-39 for buttons (no internal pull-ups there).
HONK_PIN = 18                    # horn button (D18)
BOOST_PIN = 19                   # boost button (D19)
LED_PIN = 17                     # boost LEDs (D17): GPIO -> resistor -> LED -> LED -> GND

# Cytron Robo ESP32 onboard stuff
BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

SEND_MS = 50                     # command rate (20 per second)
FIN_DEBOUNCE_MS = 4000           # ignore repeat FIN messages (car retries) for this long
# ======================================================

lights = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)
honk_btn = Pin(HONK_PIN, Pin.IN, Pin.PULL_UP)
boost_btn = Pin(BOOST_PIN, Pin.IN, Pin.PULL_UP)
boost_led = Pin(LED_PIN, Pin.OUT, value=0)


def set_lights(rgb):
    for i in range(NUM_PIXELS):
        lights[i] = rgb
    lights.write()


set_lights((0, 0, 0))


# ---------- Joystick ----------
class Axis:
    def __init__(self, pin, invert):
        self.adc = ADC(Pin(pin))
        self.adc.atten(ADC.ATTN_11DB)
        self.invert = invert
        self.center = 32768

    def raw(self):
        t = 0
        for _ in range(4):               # average a few reads to calm the noise
            t += self.adc.read_u16()
        return t // 4

    def calibrate(self):
        t = 0
        for _ in range(30):
            t += self.raw()
            time.sleep_ms(10)
        self.center = t // 30

    def value(self):
        """-1.0 .. +1.0 with deadzone."""
        v = self.raw()
        if v >= self.center:
            n = (v - self.center) / max(1, RAW_MAX - self.center)
        else:
            n = (v - self.center) / max(1, self.center - RAW_MIN)
        n = max(-1.0, min(1.0, n / THROW))
        if abs(n) < DEADZONE:
            return 0.0
        n = (abs(n) - DEADZONE) / (1.0 - DEADZONE) * (1 if n > 0 else -1)
        return -n if self.invert else n


jx = Axis(JOY_X_PIN, JOY_X_INVERT)
jy = Axis(JOY_Y_PIN, JOY_Y_INVERT)


# ---------- BLE messages ----------
# Drive command (from here):  "x,y,boost,honk\n"   e.g. "-35,100,1,0\n"  (max 14 chars, fits one packet)
# From the car:               "FIN\n"  (finish line crossed)
# From here, in reply:        "ACK\n"
rx_buf = b""
fin_flag = False


def on_data(data):
    """BLE callback: keep it short. Just collects bytes and flags a FIN message."""
    global rx_buf, fin_flag
    rx_buf += data
    if len(rx_buf) > 64:                 # safety: never let garbage pile up
        rx_buf = b""
    while b"\n" in rx_buf:
        line, rx_buf = rx_buf.split(b"\n", 1)
        if line == b"FIN":
            fin_flag = True


def send_msg(p, text):
    """Send in 20-byte pieces (the default packet size)."""
    try:
        for start in range(0, len(text), 20):
            p.send(text[start:start + 20])
            time.sleep_ms(20)
    except Exception:
        pass                              # link hiccup: the next command comes in 50 ms anyway


# ---------- Celebration ----------
def tone(freq, ms):
    buzzer = PWM(Pin(BUZZER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    buzzer.deinit()


def celebrate():
    """Victory fanfare: NeoPixels cycle colors, boost LEDs blink, buzzer plays a tune."""
    colors = ((60, 0, 0), (0, 60, 0), (0, 0, 60), (60, 40, 0))
    notes = ((523, 120), (659, 120), (784, 120), (1046, 280),
             (784, 120), (1046, 500))
    for i, (freq, ms) in enumerate(notes):
        set_lights(colors[i % len(colors)])
        boost_led.value(i % 2)
        tone(freq, ms)
        time.sleep_ms(40)
    set_lights((0, 0, 0))
    boost_led.value(0)


# ---------- Modes ----------
def test():
    print("TEST: move the joystick, press the buttons. Ctrl+C to stop.")
    print("Raw values should sit near the middle at rest and sweep close to 0 .. 65535.")
    jx.calibrate()
    jy.calibrate()
    while True:
        print("X raw=%5d val=%+.2f | Y raw=%5d val=%+.2f | honk=%d boost=%d" % (
            jx.raw(), jx.value(), jy.raw(), jy.value(),
            not honk_btn.value(), not boost_btn.value()))
        boost_led.value(not boost_btn.value())
        time.sleep_ms(200)


def session(p):
    """Runs while connected to the car."""
    global fin_flag, rx_buf
    rx_buf = b""
    fin_flag = False
    last_send = time.ticks_ms()
    last_fin = time.ticks_add(time.ticks_ms(), -FIN_DEBOUNCE_MS)

    while p.is_connected:
        # 1) Did the car report the finish line?
        if fin_flag:
            fin_flag = False
            send_msg(p, "ACK\n")                       # ACK first so the car isn't kept waiting
            now = time.ticks_ms()
            if time.ticks_diff(now, last_fin) >= FIN_DEBOUNCE_MS:
                last_fin = now
                print("Finish line crossed!")
                celebrate()
                last_send = time.ticks_ms()

        # 2) Send the current stick + button state
        now = time.ticks_ms()
        if time.ticks_diff(now, last_send) >= SEND_MS:
            last_send = now
            x = jx.value()
            y = jy.value()
            boost = not boost_btn.value()
            honk = not honk_btn.value()
            boost_led.value(boost)                     # LEDs react instantly, not via the car
            send_msg(p, "%d,%d,%d,%d\n" % (int(x * 100), int(y * 100), int(boost), int(honk)))

        time.sleep_ms(2)


def run():
    print("Hands off the joystick: calibrating center...")
    jx.calibrate()
    jy.calibrate()

    while True:                                        # reconnect automatically if the link drops
        p = None
        try:
            p = Listen(NAME, verbose=False)
            p.callback = on_data
            print("Looking for", NAME, "...")
            if p.connect_up():
                print("Connected")
                session(p)
            print("Lost connection")
        except OSError as err:
            print("BLE error:", err)
        finally:
            boost_led.value(0)
            if p:
                try:
                    p.disconnect()
                except Exception:
                    pass
        time.sleep(1)


if MODE == "test":
    test()
else:
    run()
