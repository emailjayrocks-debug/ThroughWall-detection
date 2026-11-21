# =====================================================================
#  main.py  -  Through-Wall Presence Detection  (Bits2Wave HW-4)
#  Hardware : ESP32 + DFRobot C4001 24 GHz FMCW mmWave radar + SSD1306 OLED
#  Language : MicroPython (Python that runs on the ESP32)
#
#  Signal pipeline
#   1. Radar (FMCW, 24 GHz) looks through the drywall / plywood
#   2. RANGE GATE   : everything closer than WALL_GATE_M is thrown away
#                     -> removes the strong static reflection of the wall
#   3. DOPPLER      : sensor runs in speed mode -> only targets that move
#                     (Doppler shift) are reported, static clutter is dropped
#   4. DEBOUNCE     : a target must be seen SCORE_ON times before we say
#                     "person" -> kills single-frame noise spikes
#   5. HOLD TIMER   : person stays "present" for HOLD_S after last echo
#   6. AUTO-GAIN    : signal strength is normalised to the recent peak so
#                     weak through-wall echoes still fill the bar on the OLED
#   7. CLASSIFY     : |speed| >= MICRO_SPEED -> MOVING
#                     |speed| <  MICRO_SPEED -> MICRO-MOTION (breathing/still)
# =====================================================================

from machine import UART, Pin, I2C
import time

# ------------------------- USER SETTINGS -----------------------------
MODE = "speed"        # "speed"    -> distance + speed + strength (best demo)
                      # "presence" -> also catches a person sitting dead still

UART_ID = 2
UART_RX = 16          # ESP32 GPIO16  <-  C4001 TX (green / "T")
UART_TX = 17          # ESP32 GPIO17  ->  C4001 RX (blue  / "R")
OUT_PIN = 27          # ESP32 GPIO27  <-  C4001 OUT (optional, set None if unused)
LED_PIN = 2           # on-board blue LED of most ESP32 dev boards
I2C_SDA = 21          # OLED SDA
I2C_SCL = 22          # OLED SCL

WALL_GATE_M = 0.6     # ignore echoes closer than this (the wall itself)
MAX_RANGE_M = 6.0     # how far behind the wall to look
TRIG_SENS   = 7       # 0-9, presence mode only. Higher = more sensitive
KEEP_SENS   = 7       # 0-9, presence mode only.
MICRO_SPEED = 0.15    # m/s. Slower than this = micro-movement
SCORE_ON    = 3       # consecutive hits needed to declare "person"
HOLD_S      = 3.0     # seconds to keep "person" after the last echo
NO_DATA_S   = 5.0     # warn if the radar is silent this long
DEBUG       = True    # print commands + readings to the VS Code terminal
# ---------------------------------------------------------------------


def ms():
    return time.ticks_ms()


def since(t):
    return time.ticks_diff(time.ticks_ms(), t)


# =========================== RADAR DRIVER ============================
class C4001:
    def __init__(self, uart):
        self.u = uart
        self.buf = b""

    def cmd(self, text, wait_ms=400):
        """Send a command, retrying with different line endings.
        Returns True when the radar answers 'Done'."""
        for end in ("\r\n", "\n", ""):
            self.u.read()                     # flush old bytes
            self.u.write((text + end).encode())
            time.sleep_ms(wait_ms)
            resp = (self.u.read() or b"").decode("utf-8", "ignore")
            ok = "Done" in resp
            if ok or "Error" in resp:
                break
        if DEBUG:
            r = resp.replace("\r", " ").replace("\n", " ").strip()
            print("  >", text, "|", "OK" if ok else "NO REPLY", "|", r[:50])
        return ok

    def configure(self):
        print("Configuring C4001 radar ...")
        self.ok = self.cmd("sensorStop", 600)
        self.cmd("setRunApp 1" if MODE == "speed" else "setRunApp 0")
        # Range gate inside the sensor: starts AFTER the wall
        self.cmd("setRange %.1f %.1f" % (WALL_GATE_M, MAX_RANGE_M))
        if MODE == "presence":
            self.cmd("setTrigRange %.1f" % MAX_RANGE_M)
            self.cmd("setSensitivity %d %d" % (KEEP_SENS, TRIG_SENS))
            self.cmd("setMicroMotion 1")      # detect breathing-level motion
            self.cmd("setLatency 0.1 2")      # fast trigger, 2 s keep
        self.cmd("saveConfig", 800)
        self.cmd("sensorStart", 600)
        self.buf = b""
        if not self.ok:
            print("!! Radar did not accept commands (no 'Done' reply).")
            print("!! Check ESP32 GPIO17 -> radar RX wire and common GND.")
            print("!! Radar keeps its saved settings; reading data anyway.")
        print("Requested", MODE.upper(), "mode")

    def frames(self):
        """Return all complete '$DF....*' frames received so far."""
        n = self.u.any()
        if n:
            self.buf += self.u.read(n)
        out = []
        while True:
            s = self.buf.find(b"$DF")
            if s < 0:
                self.buf = self.buf[-3:]       # keep a possible partial '$DF'
                break
            e = self.buf.find(b"*", s)
            if e < 0:
                nl = self.buf.find(b"\n", s)
                if nl < 0:
                    self.buf = self.buf[s:]
                    break
                e = nl
            out.append(self.buf[s:e].decode("utf-8", "ignore").strip())
            self.buf = self.buf[e + 1:]
        if len(self.buf) > 600:
            self.buf = b""
        return out


def parse(frame):
    """
    $DFHPD,<present>, , ,*                      (presence mode)
    $DFDMD,<num>,<x>,<range m>,<speed m/s>,<energy>, , *   (speed mode)
    Returns dict or None.
    """
    p = [x.strip() for x in frame.split(",")]
    try:
        if p[0] == "$DFHPD":
            return {"kind": "HPD", "hit": p[1] == "1", "rng": 0.0, "spd": 0.0, "en": 0}
        if p[0] == "$DFDMD":
            num = int(p[1]) if p[1] else 0
            if num < 1:
                return {"kind": "DMD", "hit": False, "rng": 0.0, "spd": 0.0, "en": 0}
            return {"kind": "DMD", "hit": True,
                    "rng": float(p[3]) if p[3] else 0.0,
                    "spd": float(p[4]) if p[4] else 0.0,
                    "en": int(float(p[5])) if p[5] else 0}
    except (IndexError, ValueError):
        pass
    return None


# ========================= DETECTION LOGIC ===========================
class Tracker:
    def __init__(self):
        self.score = 0
        self.present = False
        self.last_hit = ms()
        self.rng = 0.0        # smoothed distance
        self.spd = 0.0        # smoothed speed
        self.en = 0
        self.peak = 1.0       # auto-gain reference
        self.state = "EMPTY"
        self.kind = "DMD" if MODE == "speed" else "HPD"   # what radar really sends

    def update(self, d):
        if d is None:
            return
        self.kind = d["kind"]
        hit = d["hit"]
        # Range gate in software too (in case the sensor ignores setRange)
        if hit and self.kind == "DMD" and 0 < d["rng"] < WALL_GATE_M:
            hit = False

        if hit:
            self.score = min(self.score + 1, SCORE_ON + 5)
            if self.kind == "DMD":
                a = 0.35                                  # smoothing factor
                self.rng = d["rng"] if self.rng == 0 else (1 - a) * self.rng + a * d["rng"]
                self.spd = (1 - a) * self.spd + a * d["spd"]
                self.en = d["en"]
                self.peak = max(self.peak, float(self.en))
            if self.score >= SCORE_ON:
                self.present = True
                self.last_hit = ms()
        else:
            self.score = max(self.score - 1, 0)

    def tick(self, out_pin_high=False):
        if out_pin_high and self.kind == "HPD":
            self.present = True
            self.last_hit = ms()
        if self.present and since(self.last_hit) > HOLD_S * 1000:
            self.present = False
            self.rng = self.spd = 0.0
            self.en = 0
        self.peak = max(1.0, self.peak * 0.995)          # slow AGC decay

        if not self.present:
            self.state = "EMPTY"
        elif self.kind == "HPD":
            self.state = "OCCUPIED"
        elif abs(self.spd) >= MICRO_SPEED:
            self.state = "MOVING"
        else:
            self.state = "MICRO-MOTION"

    def strength(self):
        """0..1 auto-gained signal strength (amplified weak echoes)."""
        if not self.present or self.en <= 0:
            return 0.0
        return min(1.0, self.en / self.peak)


# ============================== DISPLAY ==============================
def init_oled():
    try:
        import ssd1306
        i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
        found = i2c.scan()
        print("I2C devices:", [hex(a) for a in found])
        if not found:
            print("No OLED found - continuing with serial output only")
            return None
        addr = 0x3C if 0x3C in found else found[0]
        return ssd1306.SSD1306_I2C(128, 64, i2c, addr=addr)
    except Exception as e:
        print("OLED disabled:", e)
        return None


def draw(oled, t, no_data):
    if oled is None:
        return
    oled.fill(0)
    oled.fill_rect(0, 0, 128, 10, 1)
    oled.text("THRU-WALL RADAR", 4, 1, 0)

    oled.text("PERSON: " + ("YES" if t.present else "NO"), 0, 14)
    oled.text(t.state, 0, 25)
    if no_data:
        oled.text("! NO RADAR DATA", 0, 40)
        oled.text("check RX wire", 0, 52)
    elif t.kind == "DMD":
        oled.text("Dist :%5.2f m" % t.rng, 0, 36)
        oled.text("Speed:%+5.2f m/s" % t.spd, 0, 46)
        w = int(t.strength() * 100)               # signal bar
        oled.text("SIG", 0, 56)
        oled.rect(26, 56, 102, 8, 1)
        oled.fill_rect(27, 57, w, 6, 1)
    else:
        oled.text("Range %.1f-%.1fm" % (WALL_GATE_M, MAX_RANGE_M), 0, 40)
        if t.present:
            oled.text("Last echo %.1fs" % (since(t.last_hit) / 1000), 0, 52)
    oled.show()


# =============================== MAIN ================================
def main():
    led = Pin(LED_PIN, Pin.OUT)
    out = Pin(OUT_PIN, Pin.IN, Pin.PULL_DOWN) if OUT_PIN is not None else None
    oled = init_oled()
    if oled:
        oled.fill(0)
        oled.text("Booting radar...", 0, 28)
        oled.show()

    uart = UART(UART_ID, baudrate=9600, tx=UART_TX, rx=UART_RX, timeout=50)
    radar = C4001(uart)
    time.sleep_ms(1500)                 # let the radar boot
    radar.configure()

    trk = Tracker()
    last_frame = ms()
    last_draw = last_print = ms()

    print("time_ms,present,state,dist_m,speed_ms,energy")
    while True:
        for f in radar.frames():
            last_frame = ms()
            trk.update(parse(f))

        trk.tick(out.value() == 1 if out else False)
        no_data = since(last_frame) > NO_DATA_S * 1000
        led.value(1 if trk.present else 0)

        if since(last_draw) > 200:
            try:
                draw(oled, trk, no_data)
            except OSError as e:                 # loose OLED wire: don't crash
                print("!! OLED error", e, "- retrying")
                time.sleep_ms(300)
                oled = init_oled()
            last_draw = ms()
        if DEBUG and since(last_print) > 500:
            if no_data:
                print("!! no data from radar - check wiring / UART switch")
            else:
                print("%d,%d,%s,%.2f,%.2f,%d" % (ms(), trk.present, trk.state,
                                                trk.rng, trk.spd, trk.en))
            last_print = ms()
        time.sleep_ms(20)


main()
