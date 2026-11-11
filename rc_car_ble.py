# rc_car.py -- MicroPython (Thonny) on ESP32 + Cytron ROBO-ESP32  -- the CAR
# Receives joystick/button packets from controller_ble.py over BLE, drives the motors,
# honks, and tells the controller when it crosses the red finish tape.
# Reuses last week's motor + VEML6040 + KNN code (needs knn_train.csv on the board).

from machine import Pin, PWM, SoftI2C
import neopixel
import time
import math
import gc
from BLE_CEEO import Yell

# "run"    -> remote-controlled car
# "train"  -> record fresh color samples into knn_train.csv (do this FIRST on the new course)
# "motors" -> test wheel direction (both should roll forward)
# "red"    -> live readout of what the color sensors see (tune finish-line detection)
MODE = "run"

# ======================= CONFIG =======================
NAME = "ESP_SY"                # BLE name: must match controller_ble.py and be unique in the class

# Motors (same as the line follower)
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True
LEFT_INVERT  = False

# Color sensors (same as the line follower)
L_SDA, L_SCL = 21, 22
R_SDA, R_SCL = 26, 25

BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

# Driving
NORMAL_SPEED = 0.50              # top speed with the joystick pushed fully
BOOST_SPEED = 1.00               # top speed while BOOST is held
MIN_DRIVE = 0.20                 # smallest duty that actually moves the car (raise if it hums but doesn't move)
HONK_FREQ = 420
FAILSAFE_MS = 300                # no command for this long -> stop the car
LOOP_MS = 40

# Finish line (red tape)
FINISH_COLOR = "RED"
FINISH_CONFIRM = 2               # consecutive detections needed (set 1 if the car skips over the tape when boosting)
FINISH_COOLDOWN_MS = 5000        # blind time after a finish so it only triggers once
ACK_TRIES = 5
ACK_WAIT_MS = 150

# KNN color detection (same values as the line follower)
KNN_K = 5
KNN_MIN_VOTES = 4
MIN_VOTES = {"RED": 4, "GREEN": 5, "BLUE": 5}
KNN_MAX_DIST = 0.12
BRIGHT_W = 0.10
RUNTIME_STRIDE = 1
TRAIN_FILE = "knn_train.csv"
SCALE = 10000
LABELS = ("WHITE", "LINE", "RED", "GREEN", "BLUE")
COLORS = ("RED", "GREEN", "BLUE")

CAL = {
    'L': (250.7, 397.0, (0.9532228, 0.84450968, 1.3041067)),
    'R': (285.1, 393.9, (0.92870264, 0.8217886, 1.4156862))
}
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
    left.set(0)
    right.set(0)


# ---------- NeoPixels ----------
np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


def np_set(r, g, b):
    for i in range(NUM_PIXELS):
        np[i] = (r, g, b)
    np.write()


def np_clear():
    np_set(0, 0, 0)


np_clear()


# ---------- Buzzer: horn + tones ----------
buzzer = None


def honk_on():
    global buzzer
    if buzzer is None:
        buzzer = PWM(Pin(BUZZER_PIN), freq=HONK_FREQ, duty_u16=32768)


def honk_off():
    global buzzer
    if buzzer is not None:
        buzzer.deinit()
        buzzer = None


def tone(freq, ms):
    b = PWM(Pin(BUZZER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    b.deinit()


# ---------- VEML6040 Driver ----------
class VEML6040:
    ADDR = 0x10

    def __init__(self, i2c, it=0):  # 40ms integration
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([(it << 4), 0x00]))

    def _rd(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def read(self):
        return self._rd(0x08), self._rd(0x09), self._rd(0x0A), self._rd(0x0B)


def make_sensors():
    i2c_l = SoftI2C(sda=Pin(L_SDA), scl=Pin(L_SCL), freq=100000)
    i2c_r = SoftI2C(sda=Pin(R_SDA), scl=Pin(R_SCL), freq=100000)
    return VEML6040(i2c_l), VEML6040(i2c_r)


def refl(w, black, white):
    if white <= black:
        return 0.0
    return max(0.0, min(1.0, (w - black) / (white - black)))


def _chroma(rgb):
    t = rgb[0] + rgb[1] + rgb[2]
    if t <= 0:
        return None
    return (rgb[0] / t, rgb[1] / t, rgb[2] / t)


def wait_loop(t0):
    rest = LOOP_MS - time.ticks_diff(time.ticks_ms(), t0)
    if rest > 0:
        time.sleep_ms(rest)


# ---------- KNN (copied from line follower v9) ----------
TRAIN = {"L": [], "R": []}
LABEL_MAP = {lab: lab for lab in LABELS}


def load_training():
    gc.collect()
    n = {"L": 0, "R": 0}
    try:
        f = open(TRAIN_FILE)
    except OSError:
        raise RuntimeError("No %s found on the board. Copy it over from the line follower." % TRAIN_FILE)
    with f:
        for line in f:
            if line[:1] in n:
                n[line[:1]] += 1
    rows = {"L": [None] * (n["L"] // RUNTIME_STRIDE), "R": [None] * (n["R"] // RUNTIME_STRIDE)}
    cnt = {"L": 0, "R": 0}
    filled = {"L": 0, "R": 0}

    with open(TRAIN_FILE) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) != 6 or p[0] not in rows or p[5] not in LABEL_MAP:
                continue
            side = p[0]
            cnt[side] += 1
            if cnt[side] % RUNTIME_STRIDE:
                continue
            k = filled[side]
            if k >= len(rows[side]):
                continue
            rows[side][k] = (int(float(p[1]) * SCALE), int(float(p[2]) * SCALE),
                             int(float(p[3]) * SCALE), int(float(p[4]) * BRIGHT_W * SCALE),
                             LABEL_MAP[p[5]])
            filled[side] = k + 1
    for side in ("L", "R"):
        if filled[side] < len(rows[side]):
            rows[side] = rows[side][:filled[side]]
    TRAIN["L"], TRAIN["R"] = rows["L"], rows["R"]
    gc.collect()
    print("Loaded KNN data: L=%d  R=%d samples (free RAM %d bytes)" % (len(TRAIN["L"]), len(TRAIN["R"]), gc.mem_free()))


def _knn_core(c0, c1, c2, q, side, skip=-1):
    train = TRAIN[side]
    if not train:
        return None, "---", 0, 9.0
    k = KNN_K
    best = []
    worst = 0x3FFFFFFF
    i = -1
    for a, b, cc, rf, lab in train:
        i += 1
        if i == skip:
            continue
        d = c0 - a
        d2 = d * d
        if d2 >= worst:
            continue
        d = c1 - b
        d2 += d * d
        if d2 >= worst:
            continue
        d = c2 - cc
        d2 += d * d
        if d2 >= worst:
            continue
        d = q - rf
        d2 += d * d
        if d2 >= worst:
            continue
        if len(best) < k:
            best.append((d2, lab))
            if len(best) == k:
                best.sort()
                worst = best[-1][0]
        else:
            best[-1] = (d2, lab)
            best.sort()
            worst = best[-1][0]
    best.sort()

    counts = {}
    for d2, lab in best:
        counts[lab] = counts.get(lab, 0) + 1
    top = max(counts.values())
    winner = None
    for d2, lab in best:
        if counts[lab] == top:
            winner = lab
            break
    ds = [math.sqrt(d2) for d2, lab in best if lab == winner]
    mean_d = sum(ds) / len(ds) / SCALE

    if winner in COLORS and top >= MIN_VOTES.get(winner, KNN_MIN_VOTES) and mean_d < KNN_MAX_DIST:
        return winner, winner, top, mean_d
    return None, winner, top, mean_d


def knn_classify(rgb, w_refl, side):
    c = _chroma(rgb)
    if c is None:
        return None, "---", 0, 9.0
    return _knn_core(int(c[0] * SCALE), int(c[1] * SCALE), int(c[2] * SCALE),
                     int(w_refl * BRIGHT_W * SCALE), side)


# ---------- Radio (BLE; the car is the "Yeller" / peripheral) ----------
# From the controller: "x,y,boost,honk\n"   e.g. "-35,100,1,0\n"
# To the controller:   "FIN\n" (finish line)   Reply from controller: "ACK\n"
rx_buf = b""
cmd = [0.0, 0.0, False, False]       # x, y, boost, honk (newest command)
last_cmd = 0                         # ticks_ms when the newest command arrived
ack_seen = False


def on_data(data):
    """BLE callback: keep it short. Parses complete lines and stores the newest command."""
    global rx_buf, last_cmd, ack_seen
    rx_buf += data
    if len(rx_buf) > 64:             # safety: never let garbage pile up
        rx_buf = b""
    while b"\n" in rx_buf:
        line, rx_buf = rx_buf.split(b"\n", 1)
        try:
            text = line.decode()
            if text == "ACK":
                ack_seen = True
                continue
            xs, ys, bs, hs = text.split(",")
            cmd[0] = int(xs) / 100.0
            cmd[1] = int(ys) / 100.0
            cmd[2] = bs == "1"
            cmd[3] = hs == "1"
            last_cmd = time.ticks_ms()
        except Exception:
            pass


def send_msg(p, text):
    """Send in 20-byte pieces (the default packet size)."""
    for start in range(0, len(text), 20):
        p.send(text[start:start + 20])
        time.sleep_ms(20)


def notify_finish(p):
    """Tell the controller we crossed the finish line; retry until it ACKs."""
    global ack_seen
    ack_seen = False
    for _ in range(ACK_TRIES):
        try:
            send_msg(p, "FIN\n")
        except Exception:
            pass
        end = time.ticks_add(time.ticks_ms(), ACK_WAIT_MS)
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            if ack_seen:
                return True
            time.sleep_ms(10)
    return False


# ---------- Driving ----------
def shape(v, cap):
    """Map -1..1 to a motor duty: 0 stays 0, anything else starts at MIN_DRIVE and tops out at cap."""
    if abs(v) < 0.02:
        return 0.0
    s = MIN_DRIVE + (cap - MIN_DRIVE) * abs(v)
    return s if v > 0 else -s


def mix(x, y, cap):
    """Arcade drive: y = forward/back, x = steer (right positive)."""
    l = y + x
    r = y - x
    m = max(abs(l), abs(r), 1.0)
    return shape(l / m, cap), shape(r / m, cap)


def celebrate_robot():
    for freq, ms in ((523, 120), (659, 120), (784, 120), (1046, 350)):
        np_set(0, 60, 0)
        tone(freq, ms)
        np_clear()
        time.sleep_ms(40)
    for _ in range(3):
        np_set(0, 60, 0)
        time.sleep_ms(120)
        np_clear()
        time.sleep_ms(120)


# ---------- Modes ----------
def train():
    """Record fresh KNN training samples and save them to knn_train.csv (overwrites it)."""
    sL, sR = make_sensors()
    N = 80                                   # samples per sensor per surface (~4 seconds each)
    plan = (
        ("WHITE", "the normal FLOOR the car will drive on", False),
        ("RED", "the RED finish-line tape (center the sensor on the tape)", False),
        ("LINE", "any OTHER surface or tape the car might cross, e.g. black tape", True),
    )
    sensors = (("L", sL, "LEFT"), ("R", sR, "RIGHT"))
    rows = 0
    with open(TRAIN_FILE, "w") as f:
        for label, desc, optional in plan:
            for side, sensor, name in sensors:
                ask = "\nPut the %s sensor over %s.\nPress Enter to record%s: " % (
                    name, desc, " (or type s + Enter to skip this one)" if optional else "")
                ans = input(ask).strip().lower()
                if optional and ans == "s":
                    break
                print("  recording %s / %s ... slide the car slowly around over that surface" % (label, name))
                for _ in range(N):
                    try:
                        w = sensor.read()
                    except OSError:
                        time.sleep_ms(50)
                        continue
                    c = _chroma(w[:3])
                    if c is None:
                        continue
                    r = refl(w[3], CAL[side][0], CAL[side][1])
                    f.write("%s,%.5f,%.5f,%.5f,%.4f,%s\n" % (side, c[0], c[1], c[2], r, label))
                    rows += 1
                    time.sleep_ms(50)
    print("\nSaved %d samples to %s." % (rows, TRAIN_FILE))
    print('Next: set MODE = "red", run it, and roll the car over the red tape to check detection.')


def motor_test():
    print("Left forward 1s, then Right forward 1s...")
    drive(0.35, 0)
    time.sleep(1)
    stop()
    time.sleep_ms(400)
    drive(0, 0.35)
    time.sleep(1)
    stop()


def red_view():
    sL, sR = make_sensors()
    load_training()
    print("Streaming live KNN data (Ctrl+C to stop). Roll the car over the red tape by hand.")
    while True:
        rl, rr = sL.read(), sR.read()
        fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
        fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
        cL, labL, vL, dL = knn_classify(rl[:3], fl, "L")
        cR, labR, vR, dR = knn_classify(rr[:3], fr, "R")
        print("L: %-5s v=%d d=%.3f [%s] | R: %-5s v=%d d=%.3f [%s]" % (
            labL, vL, dL, cL if cL else "---", labR, vR, dR, cR if cR else "---"))
        time.sleep_ms(150)


def session(p, sL, sR):
    """Runs while one controller is connected."""
    global last_cmd, rx_buf
    rx_buf = b""
    cmd[0] = cmd[1] = 0.0
    cmd[2] = cmd[3] = False
    last_cmd = time.ticks_add(time.ticks_ms(), -2 * FAILSAFE_MS)
    link = False
    hits = 0
    armed = True
    ignore_until = 0

    while p.is_connected:
        t0 = time.ticks_ms()

        # 1) Newest command (the BLE callback keeps it up to date)
        x, y, boost, honk = cmd[0], cmd[1], cmd[2], cmd[3]

        # 2) Failsafe: no commands -> stop and stay quiet
        if time.ticks_diff(t0, last_cmd) > FAILSAFE_MS:
            x = y = 0.0
            boost = honk = False
            if link:
                print("No commands - stopped")
                link = False
        elif not link:
            print("Receiving commands")
            link = True

        # 3) Drive + horn
        drive(*mix(x, y, BOOST_SPEED if boost else NORMAL_SPEED))
        if honk:
            honk_on()
        else:
            honk_off()

        # 4) Finish line (red tape)
        try:
            rl, rr = sL.read(), sR.read()
        except OSError:                       # a wobbly Grove cable shouldn't crash the run
            wait_loop(t0)
            continue
        fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
        fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
        seen = (knn_classify(rl[:3], fl, "L")[0] == FINISH_COLOR or
                knn_classify(rr[:3], fr, "R")[0] == FINISH_COLOR)
        hits = hits + 1 if seen else 0
        if not seen:
            armed = True

        if seen and armed and hits >= FINISH_CONFIRM and time.ticks_diff(t0, ignore_until) > 0:
            stop()
            honk_off()
            print("FINISH LINE!")
            got_ack = notify_finish(p)
            print("Controller ACK:", got_ack)
            celebrate_robot()
            rx_buf = b""                       # drop stale stick commands from before the finish
            cmd[0] = cmd[1] = 0.0
            cmd[2] = cmd[3] = False
            last_cmd = time.ticks_add(time.ticks_ms(), -2 * FAILSAFE_MS)   # wait for a fresh command
            armed = False
            hits = 0
            ignore_until = time.ticks_add(time.ticks_ms(), FINISH_COOLDOWN_MS)
            continue

        wait_loop(t0)


def run():
    sL, sR = make_sensors()
    load_training()
    stop()

    while True:                                # reconnect automatically if the link drops
        p = None
        try:
            p = Yell(NAME, interval_us=30000, verbose=False)
            p.callback = on_data
            print("Waiting for the controller (BLE name '%s')..." % NAME)
            if p.connect_up():
                print("Controller connected")
                session(p, sL, sR)
            print("Controller disconnected")
        except OSError as err:
            print("BLE error:", err)
        finally:
            stop()
            honk_off()
            np_clear()
            if p:
                try:
                    p.disconnect()
                except Exception:
                    pass
        time.sleep(1)


if MODE == "train":
    train()
elif MODE == "motors":
    motor_test()
elif MODE == "red":
    red_view()
else:
    run()
