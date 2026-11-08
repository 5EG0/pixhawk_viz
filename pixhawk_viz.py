#!/usr/bin/env python3
"""
Visualisasi telemetri Pixhawk dengan Pygame
===========================================
Menampilkan:
  - Artificial horizon (roll & pitch)
  - Kompas / heading
  - Model 3D wireframe quadcopter yang mengikuti attitude
  - Grafik riwayat roll / pitch / yaw
  - Info: mode, armed, GPS, altitude, kecepatan, baterai, throttle

Instalasi:
    pip install pygame pymavlink pyserial

Contoh pemakaian:
    python pixhawk_viz.py --demo                       # tanpa hardware (data simulasi)
    python pixhawk_viz.py --conn /dev/ttyACM0 --baud 115200   # Linux, USB
    python pixhawk_viz.py --conn COM5 --baud 115200           # Windows
    python pixhawk_viz.py --conn udp:127.0.0.1:14550          # SITL / Mission Planner / MAVProxy
    python pixhawk_viz.py --conn /dev/ttyUSB0 --baud 57600    # telemetry radio

Tombol: ESC = keluar
"""

import argparse
import math
import threading
import time
from collections import deque

import pygame

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

# ----------------------------------------------------------------------------
# Konfigurasi tampilan
# ----------------------------------------------------------------------------
W, H = 1100, 700
FPS = 60

BG = (18, 20, 26)
PANEL = (28, 31, 40)
BORDER = (60, 66, 82)
TEXT = (225, 228, 235)
MUTED = (140, 146, 162)
SKY = (52, 126, 200)
GROUND = (128, 88, 52)
GREEN = (80, 220, 120)
RED = (235, 80, 80)
YELLOW = (250, 200, 60)
CYAN = (70, 210, 230)
ORANGE = (255, 150, 50)


# ----------------------------------------------------------------------------
# Data telemetri (dibagi antara thread pembaca dan thread render)
# ----------------------------------------------------------------------------
class Telemetry:
    def __init__(self):
        self.roll = 0.0       # derajat
        self.pitch = 0.0      # derajat
        self.yaw = 0.0        # derajat (0..360)
        self.heading = 0.0    # derajat dari VFR_HUD
        self.alt = 0.0        # m (relatif)
        self.groundspeed = 0.0
        self.airspeed = 0.0
        self.climb = 0.0
        self.throttle = 0
        self.volt = 0.0
        self.curr = 0.0
        self.batt_pct = -1
        self.gps_fix = 0
        self.sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.armed = False
        self.mode = "-"
        self.last_heartbeat = 0.0
        self.status = "Memulai..."


# ----------------------------------------------------------------------------
# Sumber data: MAVLink (Pixhawk asli)
# ----------------------------------------------------------------------------
def mavlink_worker(conn, baud, tel, stop):
    if mavutil is None:
        tel.status = "pymavlink belum terpasang (pip install pymavlink)"
        return
    try:
        tel.status = f"Menghubungkan ke {conn} ..."
        m = mavutil.mavlink_connection(conn, baud=baud)
    except Exception as e:  # noqa: BLE001
        tel.status = f"Gagal koneksi: {e}"
        return

    tel.status = "Menunggu heartbeat ..."
    while not stop.is_set():
        if m.wait_heartbeat(timeout=2):
            break
    if stop.is_set():
        return

    tel.status = f"Terhubung (sys {m.target_system}, comp {m.target_component})"
    # Minta semua data stream pada 10 Hz
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    while not stop.is_set():
        try:
            msg = m.recv_match(blocking=True, timeout=1)
        except Exception as e:  # noqa: BLE001
            tel.status = f"Error baca: {e}"
            time.sleep(0.5)
            continue
        if msg is None:
            continue
        t = msg.get_type()

        if t == "ATTITUDE":
            tel.roll = math.degrees(msg.roll)
            tel.pitch = math.degrees(msg.pitch)
            tel.yaw = math.degrees(msg.yaw) % 360
        elif t == "VFR_HUD":
            tel.heading = msg.heading
            tel.groundspeed = msg.groundspeed
            tel.airspeed = msg.airspeed
            tel.climb = msg.climb
            tel.throttle = msg.throttle
        elif t == "GLOBAL_POSITION_INT":
            tel.alt = msg.relative_alt / 1000.0
            tel.lat = msg.lat / 1e7
            tel.lon = msg.lon / 1e7
        elif t == "GPS_RAW_INT":
            tel.gps_fix = msg.fix_type
            tel.sats = msg.satellites_visible
        elif t == "SYS_STATUS":
            tel.volt = msg.voltage_battery / 1000.0
            tel.curr = msg.current_battery / 100.0 if msg.current_battery >= 0 else 0.0
            tel.batt_pct = msg.battery_remaining
        elif t == "HEARTBEAT":
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            tel.last_heartbeat = time.time()
            tel.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                tel.mode = mavutil.mode_string_v10(msg)
            except Exception:  # noqa: BLE001
                tel.mode = str(msg.custom_mode)


# ----------------------------------------------------------------------------
# Sumber data: simulasi (tanpa hardware)
# ----------------------------------------------------------------------------
def demo_worker(tel, stop):
    tel.status = "MODE DEMO (data simulasi)"
    t0 = time.time()
    while not stop.is_set():
        t = time.time() - t0
        tel.roll = 30 * math.sin(t * 0.8)
        tel.pitch = 15 * math.sin(t * 0.5 + 1)
        tel.yaw = (t * 20) % 360
        tel.heading = tel.yaw
        tel.alt = 25 + 5 * math.sin(t * 0.3)
        tel.groundspeed = 8 + 2 * math.sin(t * 0.4)
        tel.airspeed = tel.groundspeed
        tel.climb = 1.5 * math.cos(t * 0.3)
        tel.throttle = int(50 + 20 * math.sin(t * 0.6))
        tel.volt = max(10.5, 12.6 - t * 0.005)
        tel.curr = 15 + 3 * math.sin(t)
        tel.batt_pct = int(max(0, 100 - t * 0.2))
        tel.gps_fix = 3
        tel.sats = 12
        tel.lat = -6.9667 + 0.0001 * math.sin(t * 0.1)
        tel.lon = 110.4167 + 0.0001 * math.cos(t * 0.1)
        tel.armed = True
        tel.mode = "STABILIZE"
        tel.last_heartbeat = time.time()
        time.sleep(0.02)


# ----------------------------------------------------------------------------
# Fungsi gambar
# ----------------------------------------------------------------------------
_mask_cache = {}


def circle_mask(r):
    if r not in _mask_cache:
        mask = pygame.Surface((2 * r, 2 * r), pygame.SRCALPHA)
        pygame.draw.circle(mask, (255, 255, 255, 255), (r, r), r)
        _mask_cache[r] = mask
    return _mask_cache[r]


def draw_panel(screen, rect, title, font):
    pygame.draw.rect(screen, PANEL, rect, border_radius=10)
    pygame.draw.rect(screen, BORDER, rect, 1, border_radius=10)
    screen.blit(font.render(title, True, MUTED), (rect.x + 12, rect.y + 8))


def draw_horizon(screen, cx, cy, r, roll, pitch, font_s):
    """Artificial horizon: langit/tanah yang berotasi sesuai roll & pitch."""
    pitch_c = max(-60.0, min(60.0, pitch))
    ppd = r / 35.0                      # piksel per derajat
    size = 4 * r
    c = size // 2
    big = pygame.Surface((size, size))
    big.fill(SKY)
    hy = c + pitch_c * ppd              # pitch naik -> horizon turun
    pygame.draw.rect(big, GROUND, (0, hy, size, size - hy))
    pygame.draw.line(big, (255, 255, 255), (0, hy), (size, hy), 3)

    # Tangga pitch
    for p in range(-30, 31, 10):
        if p == 0:
            continue
        y = hy - p * ppd
        half = 40 if p % 20 == 0 else 22
        pygame.draw.line(big, (255, 255, 255), (c - half, y), (c + half, y), 2)
        lbl = font_s.render(str(abs(p)), True, (255, 255, 255))
        big.blit(lbl, (c + half + 6, y - lbl.get_height() // 2))
        big.blit(lbl, (c - half - 6 - lbl.get_width(), y - lbl.get_height() // 2))

    rot = pygame.transform.rotate(big, roll)   # positif = CCW (sayap kanan turun)
    view = pygame.Surface((2 * r, 2 * r), pygame.SRCALPHA)
    view.blit(rot, rot.get_rect(center=(r, r)))
    view.blit(circle_mask(r), (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    screen.blit(view, (cx - r, cy - r))

    # Cincin & skala roll
    pygame.draw.circle(screen, (235, 235, 240), (cx, cy), r, 3)
    for a in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
        ang = math.radians(90 + a)
        l = 14 if a % 30 == 0 else 8
        x1, y1 = cx + r * math.cos(ang), cy - r * math.sin(ang)
        x2, y2 = cx + (r - l) * math.cos(ang), cy - (r - l) * math.sin(ang)
        pygame.draw.line(screen, (255, 255, 255), (x1, y1), (x2, y2), 2)

    # Penunjuk roll (segitiga) ikut berotasi bersama horizon
    ang = math.radians(90 + roll)
    px, py = cx + (r - 18) * math.cos(ang), cy - (r - 18) * math.sin(ang)
    tip = (cx + (r - 3) * math.cos(ang), cy - (r - 3) * math.sin(ang))
    nx, ny = -math.sin(ang), -math.cos(ang)
    pygame.draw.polygon(screen, YELLOW, [tip, (px + nx * 8, py + ny * 8), (px - nx * 8, py - ny * 8)])

    # Simbol pesawat tetap di tengah
    pygame.draw.line(screen, YELLOW, (cx - 70, cy), (cx - 22, cy), 5)
    pygame.draw.line(screen, YELLOW, (cx + 22, cy), (cx + 70, cy), 5)
    pygame.draw.line(screen, YELLOW, (cx - 22, cy), (cx - 22, cy + 12), 5)
    pygame.draw.line(screen, YELLOW, (cx + 22, cy), (cx + 22, cy + 12), 5)
    pygame.draw.circle(screen, YELLOW, (cx, cy), 5)


def draw_compass(screen, cx, cy, r, heading, font, font_s):
    pygame.draw.circle(screen, (14, 16, 22), (cx, cy), r)
    pygame.draw.circle(screen, (235, 235, 240), (cx, cy), r, 2)
    names = {0: "N", 90: "E", 180: "S", 270: "W"}
    for b in range(0, 360, 10):
        rel = math.radians(b - heading)
        s, c = math.sin(rel), -math.cos(rel)
        l = 14 if b % 30 == 0 else 7
        pygame.draw.line(screen, (200, 200, 210),
                         (cx + (r - 2) * s, cy + (r - 2) * c),
                         (cx + (r - 2 - l) * s, cy + (r - 2 - l) * c), 2)
        if b in names:
            lbl = font.render(names[b], True, RED if b == 0 else TEXT)
            lx, ly = cx + (r - 30) * s, cy + (r - 30) * c
            screen.blit(lbl, (lx - lbl.get_width() / 2, ly - lbl.get_height() / 2))
    # Jarum tetap menunjuk ke atas (arah hidung)
    pygame.draw.polygon(screen, YELLOW, [(cx, cy - r + 36), (cx - 9, cy + 6), (cx + 9, cy + 6)])
    pygame.draw.circle(screen, BG, (cx, cy), 4)
    txt = font.render(f"{int(heading) % 360:03d}\u00b0", True, TEXT)
    screen.blit(txt, (cx - txt.get_width() // 2, cy + 22))


# ---- Model 3D wireframe ---------------------------------------------------
def build_quad():
    """Titik-titik model quad (X maju, Y kanan, Z bawah)."""
    verts, edges, colors = [(0, 0, 0)], [], []
    arm = 1.0
    for i, ang in enumerate((45, 135, 225, 315)):
        a = math.radians(ang)
        x, y = arm * math.cos(a), arm * math.sin(a)
        verts.append((x, y, 0))
        edges.append((0, len(verts) - 1))
        colors.append(RED if ang in (45, 315) else (230, 230, 235))  # lengan depan merah
    verts.append((1.6, 0, 0))             # hidung
    edges.append((0, len(verts) - 1))
    colors.append(GREEN)
    return verts, edges, colors


QUAD = build_quad()


def rotate_ned(v, roll, pitch, yaw):
    x, y, z = v
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    x1, y1, z1 = x, y * cr - z * sr, y * sr + z * cr
    x2, y2, z2 = x1 * cp + z1 * sp, y1, -x1 * sp + z1 * cp
    return x2 * cy - y2 * sy, x2 * sy + y2 * cy, z2


def project(n, e, d, cx, cy, scale):
    up = -d
    return cx + (e + 0.35 * n) * scale, cy - (up * 0.9 + 0.45 * n) * scale


def draw_3d(screen, rect, tel, font_s):
    cx, cy = rect.centerx, rect.centery + 10
    scale = 80
    # Grid lantai
    for i in range(-3, 4):
        a = project(i * 0.8, -2.4, 1.2, cx, cy, scale)
        b = project(i * 0.8, 2.4, 1.2, cx, cy, scale)
        pygame.draw.line(screen, (45, 50, 64), a, b, 1)
        a = project(-2.4, i * 0.8, 1.2, cx, cy, scale)
        b = project(2.4, i * 0.8, 1.2, cx, cy, scale)
        pygame.draw.line(screen, (45, 50, 64), a, b, 1)

    roll, pitch, yaw = (math.radians(tel.roll), math.radians(tel.pitch), math.radians(tel.yaw))
    verts, edges, colors = QUAD
    pts = [project(*rotate_ned(v, roll, pitch, yaw), cx, cy, scale) for v in verts]
    for (a, b), col in zip(edges, colors):
        pygame.draw.line(screen, col, pts[a], pts[b], 4)
    for i in range(1, 5):                          # motor
        pygame.draw.circle(screen, ORANGE, (int(pts[i][0]), int(pts[i][1])), 9, 2)
    pygame.draw.circle(screen, CYAN, (int(pts[0][0]), int(pts[0][1])), 6)
    pygame.draw.circle(screen, GREEN, (int(pts[-1][0]), int(pts[-1][1])), 5)

    # Label sumbu NED kecil
    lbl = font_s.render("merah = depan (arm), hijau = hidung", True, MUTED)
    screen.blit(lbl, (rect.x + 12, rect.bottom - 24))


def draw_graph(screen, rect, hist, font_s):
    gx, gy, gw, gh = rect.x + 12, rect.y + 34, rect.w - 24, rect.h - 70
    pygame.draw.rect(screen, (16, 18, 24), (gx, gy, gw, gh))
    mid = gy + gh // 2
    pygame.draw.line(screen, (60, 66, 82), (gx, mid), (gx + gw, mid), 1)
    series = (("Roll", hist["roll"], RED), ("Pitch", hist["pitch"], GREEN), ("Yaw", hist["yaw"], CYAN))
    for name, data, col in series:
        if len(data) < 2:
            continue
        pts = []
        for i, v in enumerate(data):
            x = gx + i * gw / (data.maxlen - 1)
            v = max(-90, min(90, v))
            pts.append((x, mid - v / 90.0 * (gh / 2 - 4)))
        pygame.draw.lines(screen, col, False, pts, 2)
    x = gx
    for name, _, col in series:
        t = font_s.render(name, True, col)
        screen.blit(t, (x, gy + gh + 8))
        x += t.get_width() + 18
    screen.blit(font_s.render("\u00b190\u00b0", True, MUTED), (gx + gw - 40, gy + gh + 8))


def draw_bar(screen, rect, frac, color, label, font_s):
    pygame.draw.rect(screen, (16, 18, 24), rect, border_radius=5)
    fill = rect.copy()
    fill.w = int(rect.w * max(0.0, min(1.0, frac)))
    pygame.draw.rect(screen, color, fill, border_radius=5)
    pygame.draw.rect(screen, BORDER, rect, 1, border_radius=5)
    t = font_s.render(label, True, TEXT)
    screen.blit(t, (rect.x + 6, rect.centery - t.get_height() // 2))


def draw_info(screen, rect, tel, fonts):
    font, font_s, font_b = fonts
    x, y = rect.x + 16, rect.y + 36
    link_ok = (time.time() - tel.last_heartbeat) < 3.0

    def row(label, value, color=TEXT):
        nonlocal y
        screen.blit(font_s.render(label, True, MUTED), (x, y))
        v = font.render(value, True, color)
        screen.blit(v, (rect.right - 16 - v.get_width(), y - 2))
        y += 28

    row("Link", "OK" if link_ok else "PUTUS", GREEN if link_ok else RED)
    row("Mode", tel.mode, CYAN)
    row("Status", "ARMED" if tel.armed else "DISARMED", RED if tel.armed else GREEN)
    y += 8
    fixes = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
    row("GPS", fixes.get(tel.gps_fix, str(tel.gps_fix)), GREEN if tel.gps_fix >= 3 else YELLOW)
    row("Satelit", str(tel.sats))
    row("Lat", f"{tel.lat:.6f}")
    row("Lon", f"{tel.lon:.6f}")
    y += 8
    row("Altitude", f"{tel.alt:6.1f} m")
    row("Ground speed", f"{tel.groundspeed:5.1f} m/s")
    row("Air speed", f"{tel.airspeed:5.1f} m/s")
    row("Climb", f"{tel.climb:+5.1f} m/s")
    y += 8
    row("Roll", f"{tel.roll:+6.1f}\u00b0")
    row("Pitch", f"{tel.pitch:+6.1f}\u00b0")
    row("Yaw", f"{tel.yaw:6.1f}\u00b0")
    y += 8
    row("Baterai", f"{tel.volt:5.2f} V  {tel.curr:4.1f} A")
    pct = tel.batt_pct
    col = GREEN if pct > 40 else YELLOW if pct > 20 else RED
    draw_bar(screen, pygame.Rect(x, y, rect.w - 32, 20),
             max(pct, 0) / 100.0, col if pct >= 0 else BORDER,
             f"{pct}%" if pct >= 0 else "N/A", font_s)
    y += 32
    draw_bar(screen, pygame.Rect(x, y, rect.w - 32, 20), tel.throttle / 100.0, ORANGE,
             f"Throttle {tel.throttle}%", font_s)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Visualisasi Pixhawk dengan Pygame")
    ap.add_argument("--conn", default="udp:127.0.0.1:14550",
                    help="Connection string, mis. /dev/ttyACM0, COM5, udp:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate (untuk serial)")
    ap.add_argument("--demo", action="store_true", help="Pakai data simulasi (tanpa hardware)")
    args = ap.parse_args()

    tel = Telemetry()
    stop = threading.Event()
    if args.demo:
        target = threading.Thread(target=demo_worker, args=(tel, stop), daemon=True)
    else:
        target = threading.Thread(target=mavlink_worker, args=(args.conn, args.baud, tel, stop), daemon=True)
    target.start()

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Pixhawk Visualizer")
    clock = pygame.time.Clock()

    mono = "consolas,dejavusansmono,menlo,couriernew,monospace"
    font = pygame.font.SysFont(mono, 18, bold=True)
    font_s = pygame.font.SysFont(mono, 14)
    font_b = pygame.font.SysFont(mono, 22, bold=True)
    fonts = (font, font_s, font_b)

    hist = {k: deque(maxlen=300) for k in ("roll", "pitch", "yaw")}
    r_hor = pygame.Rect(20, 40, 390, 390)
    r_cmp = pygame.Rect(20, 445, 390, 235)
    r_3d = pygame.Rect(425, 40, 345, 340)
    r_gr = pygame.Rect(425, 395, 345, 285)
    r_info = pygame.Rect(785, 40, 295, 640)

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False

        hist["roll"].append(tel.roll)
        hist["pitch"].append(tel.pitch)
        yaw_n = tel.yaw if tel.yaw <= 180 else tel.yaw - 360
        hist["yaw"].append(yaw_n / 2.0)       # diskalakan agar muat di grafik

        screen.fill(BG)
        screen.blit(font_b.render("PIXHAWK VISUALIZER", True, TEXT), (20, 8))
        st = font_s.render(tel.status, True, MUTED)
        screen.blit(st, (W - st.get_width() - 20, 14))

        draw_panel(screen, r_hor, "ATTITUDE", font_s)
        draw_horizon(screen, r_hor.centerx, r_hor.centery + 10, 160, tel.roll, tel.pitch, font_s)

        draw_panel(screen, r_cmp, "HEADING", font_s)
        draw_compass(screen, r_cmp.centerx, r_cmp.centery + 10, 95, tel.heading, font, font_s)

        draw_panel(screen, r_3d, "MODEL 3D", font_s)
        draw_3d(screen, r_3d, tel, font_s)

        draw_panel(screen, r_gr, "GRAFIK ROLL / PITCH / YAW", font_s)
        draw_graph(screen, r_gr, hist, font_s)

        draw_panel(screen, r_info, "TELEMETRI", font_s)
        draw_info(screen, r_info, tel, fonts)

        pygame.display.flip()
        clock.tick(FPS)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()