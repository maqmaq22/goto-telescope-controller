import sys
sys.path.append('/home/goto/Downloads/UPS_Module_3S_Code/RaspberryPi/UPS Module 3S')

try:
    from INA219 import INA219
    ups_sensor_ok = True
except ImportError:
    print('Błąd: Nie znaleziono INA219.py')
    ups_sensor_ok = False

import os, time, threading, requests
import cv2
import numpy as np
from flask import Flask, render_template, Response, send_from_directory, request, jsonify
from flask_socketio import SocketIO
from skyfield.api import load, wgs84, EarthSatellite, Star
from HR8825 import HR8825

# GPIO do fizycznego czujnika kolizji. Na komputerze bez Raspberry Pi kod nadal się uruchomi.
try:
    import RPi.GPIO as GPIO
    GPIO_OK = True
except Exception as e:
    print(f'UWAGA: RPi.GPIO niedostępne: {e}')
    GPIO = None
    GPIO_OK = False



# ============================================================
# ASTRO GPS — wyznaczanie lokalizacji z pozycji gwiazd/Słońca
# ============================================================

astro_gps_observations = []

def dodaj_obserwacje_astro_gps(obiekt, observed_alt, observed_az):
    """
    Dodaje obserwację do systemu Astro GPS.

    observed_alt = wysokość nad horyzontem
    observed_az  = azymut
    """

    astro_gps_observations.append({
        "object": obiekt,
        "alt": observed_alt,
        "az": observed_az,
        "time": ts.now()
    })

def policz_lokalizacje_z_gwiazd():
    """
    Próbuje wyznaczyć pozycję geograficzną
    na podstawie obserwacji nieba.
    """

    if len(astro_gps_observations) < 2:
        return False, "Potrzeba minimum 2 obserwacji"

    best_error = 999999999
    best_lat = None
    best_lon = None

    # Prosty brute force dla Polski
    for lat in [x / 10 for x in range(490, 560)]:
        for lon in [x / 10 for x in range(140, 250)]:

            try:
                test_place = earth + wgs84.latlon(
                    lat,
                    lon,
                    elevation_m=wysokosc_m
                )

                total_error = 0

                for obs in astro_gps_observations:

                    obj_name = obs["object"].lower()

                    if obj_name == "sun":
                        body = eph['sun']
                    elif obj_name == "saturn":
                        body = eph['saturn barycenter']
                    elif obj_name == "moon":
                        body = eph['moon']
                    else:
                        continue

                    astrometric = test_place.at(obs["time"]).observe(body)
                    apparent = astrometric.apparent()

                    alt, az, dist = apparent.altaz()

                    d_alt = abs(alt.degrees - obs["alt"])
                    d_az = abs(az.degrees - obs["az"])

                    total_error += d_alt + d_az

                if total_error < best_error:
                    best_error = total_error
                    best_lat = lat
                    best_lon = lon

            except Exception:
                pass

    if best_lat is None:
        return False, "Nie udało się policzyć lokalizacji"

    return True, {
        "latitude": best_lat,
        "longitude": best_lon,
        "error": round(best_error, 3),
        "samples": len(astro_gps_observations)
    }





# ============================================================
# OPENCV OBJECT TRACKING
# ============================================================

import cv2
import threading
import time
import numpy as np

vision_tracking_enabled = False
vision_tracking_thread = None

vision_tracking_state = {
    "target_x": 0,
    "target_y": 0,
    "frame_width": 0,
    "frame_height": 0,
    "tracking": False,
    "last_seen": False
}

def vision_tracking_loop():

    global vision_tracking_enabled

    print("Vision tracking started")

    while vision_tracking_enabled:

        ret, frame = read_camera_frame()

        if not ret or frame is None:
            time.sleep(0.1)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 0)

        # Dynamiczny próg: lepiej działa dla Słońca/Księżyca/planet niż stałe 180.
        max_val = float(np.max(gray)) if gray.size else 0.0
        threshold_value = max(80, min(245, int(max_val * 0.70)))
        _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h, w = gray.shape
        center_x = w // 2
        center_y = h // 2

        vision_tracking_state["frame_width"] = w
        vision_tracking_state["frame_height"] = h

        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 20:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    dx = cx - center_x
                    dy = cy - center_y

                    vision_tracking_state["target_x"] = cx
                    vision_tracking_state["target_y"] = cy
                    vision_tracking_state["tracking"] = True
                    vision_tracking_state["last_seen"] = True

                    tolerance = 25
                    if obsluz_kolizje_gpio('vision'):
                        awaryjne_ominiecie_kolizji()
                        time.sleep(0.2)
                        continue
                    if abs(dx) > tolerance:
                        move_x(1 if dx > 0 else -1)
                    if abs(dy) > tolerance:
                        move_y(-1 if dy > 0 else 1)
                else:
                    vision_tracking_state["tracking"] = False
            else:
                vision_tracking_state["tracking"] = False
        else:
            vision_tracking_state["tracking"] = False
            vision_tracking_state["last_seen"] = False

        time.sleep(0.05)

    vision_tracking_state["tracking"] = False
    print("Vision tracking stopped")

def start_vision_tracking():

    global vision_tracking_enabled
    global vision_tracking_thread

    if vision_tracking_enabled:
        return False

    vision_tracking_enabled = True

    vision_tracking_thread = threading.Thread(
        target=vision_tracking_loop,
        daemon=True
    )

    vision_tracking_thread.start()

    return True

def stop_vision_tracking():

    global vision_tracking_enabled

    vision_tracking_enabled = False

    return True



app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

folder_zapisu = "/home/goto/Pictures/Teleskop"
os.makedirs(folder_zapisu, exist_ok=True)

AUTO_CAMERA = True
CAMERA_INDEX = 0
CAMERA_SCAN_INDICES = [0, 1, 2, 3, 4]
KAMERA_SZEROKOSC = 640
KAMERA_WYSOKOSC = 480
KAMERA_FPS = 30
LOCATION_MODE_LABEL = 'z kodu'

kierunek_x_plus = 'backward'
kierunek_x_minus = 'forward'
kierunek_y_plus = 'backward'
kierunek_y_minus = 'forward'

opoznienie_kroku = 0.00035
wielkosc_paczki_auto = 150
reczne_kroki_x = 100
reczne_kroki_y = 100
manual_loop_delay = 0.0

# Prędkość ruchu automatycznego / GoTo ustawiana suwakiem w interfejsie
auto_step_delay_default = 0.00035
auto_step_chunk_default = 150

min_x, max_x = -3000000, 3000000
min_y, max_y = -2000000, 2000000

szerokosc = 54.52
dlugosc = 18.53
wysokosc_m = 30

kroki_x_na_360 = 2496000
kroki_y_na_360 = 1248000
kroki_x_na_1_godzine_ra = kroki_x_na_360 / 24
kroki_y_na_1_stopien_dec = kroki_y_na_360 / 360

polarna_ra = 2 + 31 / 60 + 49 / 3600
polarna_dec = 89 + 15 / 60 + 51 / 3600

czas_odswiezania_planety = 1.0  # częstsze korekty, żeby po kalibracji Słońce/Jowisz realnie się prowadziły
czas_odswiezania_iss = 1.0

PROG_KOLIZJI_A = -0.0430  # obecnie NIE zatrzymuje ruchu; tylko telemetria INA219
ILOSC_PROBEK_KOLIZJI = 5
MIN_TRAFIEN_DO_KOLIZJI = 4
ODSTEP_PROBEK_KOLIZJI = 0.004
KOLIZJA_COOLDOWN_S = 0.7
CZAS_BANNERA_KOLIZJI_S = 3.0
POKAZUJ_DIAGNOSTYKE_KOLIZJI = True

# Fizyczny czujnik kolizji: wejście z podciąganiem do góry, aktywne po zwarciu do GND.
# UWAGA: GPIO 21 w tym projekcie jest zajęte przez silnik Y jako mode_pin.
# Dlatego domyślnie używam GPIO 26. Jeśli naprawdę chcesz GPIO 21, przepnij mode_pin silnika
# albo ustaw zmienną środowiskową PIN_KOLIZJI=21 i sprawdź okablowanie.
PIN_KOLIZJI = int(os.environ.get('PIN_KOLIZJI', '26'))
KOLIZJA_GPIO_AKTYWNA = os.environ.get('KOLIZJA_GPIO_AKTYWNA', '1') != '0'
KROKI_COFANIA_KOLIZJI_X = int(os.environ.get('KROKI_COFANIA_KOLIZJI_X', '350'))
KROKI_OMIJANIA_KOLIZJI_Y = int(os.environ.get('KROKI_OMIJANIA_KOLIZJI_Y', '180'))
CZAS_PO_OMINIECIU_KOLIZJI_S = float(os.environ.get('CZAS_PO_OMINIECIU_KOLIZJI_S', '0.25'))

state_lock = threading.Lock()
camera_lock = threading.Lock()
ina_lock = threading.Lock()
motor_lock = threading.Lock()
ina_sensor = None
camera = None

silnik_x = HR8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
silnik_y = HR8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))
silnik_x.SetMicroStep('softward', '1/32step')
silnik_y.SetMicroStep('softward', '1/32step')

def init_gpio_kolizji():
    if not GPIO_OK or not KOLIZJA_GPIO_AKTYWNA:
        print('Czujnik kolizji GPIO: wyłączony albo brak RPi.GPIO')
        return False
    zajete_piny = {13, 19, 12, 16, 17, 20, 24, 18, 4, 21, 22, 27}
    if PIN_KOLIZJI in zajete_piny:
        print(f'UWAGA: PIN_KOLIZJI={PIN_KOLIZJI} jest też użyty przez sterownik silnika. Sprawdź okablowanie!')
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN_KOLIZJI, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f'Czujnik kolizji GPIO aktywny na BCM {PIN_KOLIZJI}')
        return True
    except Exception as e:
        print(f'Błąd inicjalizacji czujnika kolizji GPIO: {e}')
        return False

gpio_kolizja_ok = init_gpio_kolizji()

ts = load.timescale()
eph = load('de421.bsp')
earth = eph['earth']
sun = eph['sun']
moon = eph['moon']
miejsce = earth + wgs84.latlon(szerokosc, dlugosc, elevation_m=wysokosc_m)

BRIGHT_STARS = {
    'sirius': Star(ra_hours=6 + 45/60 + 8.917/3600, dec_degrees=-(16 + 42/60 + 58.02/3600)),
    'vega': Star(ra_hours=18 + 36/60 + 56.336/3600, dec_degrees=38 + 47/60 + 1.28/3600),
    'capella': Star(ra_hours=5 + 16/60 + 41.359/3600, dec_degrees=45 + 59/60 + 52.77/3600),
    'arcturus': Star(ra_hours=14 + 15/60 + 39.672/3600, dec_degrees=19 + 10/60 + 56.67/3600),
    'rigel': Star(ra_hours=5 + 14/60 + 32.272/3600, dec_degrees=-(8 + 12/60 + 5.90/3600)),
    'betelgeuse': Star(ra_hours=5 + 55/60 + 10.305/3600, dec_degrees=7 + 24/60 + 25.43/3600),
    'procyon': Star(ra_hours=7 + 39/60 + 18.119/3600, dec_degrees=5 + 13/60 + 29.96/3600),
    'aldebaran': Star(ra_hours=4 + 35/60 + 55.239/3600, dec_degrees=16 + 30/60 + 33.49/3600),
    'altair': Star(ra_hours=19 + 50/60 + 47.004/3600, dec_degrees=8 + 52/60 + 5.96/3600),
    'deneb': Star(ra_hours=20 + 41/60 + 25.916/3600, dec_degrees=45 + 16/60 + 49.22/3600),
}
OBJECT_LABELS = {'sun':'Słońce','moon':'Księżyc','mercury':'Merkury','venus':'Wenus','mars':'Mars','jupiter':'Jowisz','saturn':'Saturn','uranus':'Uran','neptune':'Neptun','iss':'ISS','sirius':'Syriusz','vega':'Wega','capella':'Kapella','arcturus':'Arktur','rigel':'Rigel','betelgeuse':'Betelgeza','procyon':'Procjon','aldebaran':'Aldebaran','altair':'Altair','deneb':'Deneb'}

state = {
    'czy_sledzic': False, 'obiekt_cel': None, 'nazwa_obiektu': 'Brak',
    'status': "Gotowe. Możesz użyć kalibracji 3-punktowej albo awaryjnie ustawić Polarną.",
    'aktualne_ra': 0.0, 'aktualne_dec': 0.0, 'aktualne_kroki_x': 0, 'aktualne_kroki_y': 0,
    'czy_iss': False, 'mode': 'auto', 'manual_direction': None, 'manual_step_delay': 0.00015,
    'battery': 0.0, 'camera_ok': False, 'camera_index': None, 'current_A': 0.0, 'voltage_V': 0.0,
    'collision_alert': False, 'collision_axis': None, 'collision_event_id': 0, 'last_collision_time': 0.0,
    'gpio_collision': False, 'gpio_collision_pin': PIN_KOLIZJI,
    'location_mode': LOCATION_MODE_LABEL, 'calibration_points': [], 'calibrated': False,
    'calibration_offset_x': 0, 'calibration_offset_y': 0,
    # Model kalibracji 3-punktowej: affine RA/DEC -> kroki silników.
    # Pozwala pracować bez ustawiania na Polarną, po ręcznym zatwierdzeniu 3 obiektów.
    'calibration_model': None, 'calibration_ra_ref': None,
    # Nowy poprawny model do pracy bez Polarnej: lokalne ALT/AZ -> kroki silników.
    # ALT/AZ zmienia się wraz z obrotem Ziemi, więc działa też dla 3 punktów tego samego obiektu w czasie.
    'calibration_altaz_model': None, 'calibration_az_ref': None,
    'aktualne_alt': None, 'aktualne_az': None,
    'auto_step_delay': auto_step_delay_default, 'auto_step_chunk': auto_step_chunk_default,
    'auto_speed_percent': 50,
    # Prowadzenie wyliczone z kalibracji czasowej, np. 3x Słońce.
    # To jest realna prędkość silników potrzebna, żeby obiekt nie uciekał z kadru.
    'guide_rate_x': 0.0,
    'guide_rate_y': 0.0,
    'guide_rate_ready': False,
    'guide_source': None,
    'guide_anchor_time': None,
    'guide_anchor_steps_x': None,
    'guide_anchor_steps_y': None,
    'guide_anchor_object': None,
    'obiekt_cel_name': None,
    'goto_pending': False,
}

def pobierz_iss():
    try:
        tekst = requests.get('https://celestrak.org/NORAD/elements/stations.txt', timeout=10).text.splitlines()
        for i, line in enumerate(tekst):
            if 'ISS' in line.strip().upper() and i + 2 < len(tekst):
                print('ISS: pobrano z internetu')
                return EarthSatellite(tekst[i+1].strip(), tekst[i+2].strip(), 'ISS', ts)
        raise Exception('Brak ISS')
    except Exception:
        print('UWAGA: używam zapisanej orbity ISS')
        return EarthSatellite('1 25544U 98067A 24070.54791667 .00016717 00000+0 10270-3 0 9991', '2 25544 51.6421 21.5623 0004517 88.2410 39.4512 15.50000012 34567', 'ISS', ts)
iss = pobierz_iss()

def make_placeholder_frame(text):
    frame = np.zeros((KAMERA_WYSOKOSC, KAMERA_SZEROKOSC, 3), dtype=np.uint8)
    cv2.putText(frame, 'BRAK KAMERY', (150, 210), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255,255,255), 3)
    cv2.putText(frame, text[:45], (30, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)
    return frame

def try_open_camera(index):
    print(f'[KAMERA] Próba /dev/video{index}')
    cam = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cam.isOpened():
        cam.release(); print(f'[KAMERA] /dev/video{index} nie działa'); return None
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, KAMERA_SZEROKOSC)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, KAMERA_WYSOKOSC)
    cam.set(cv2.CAP_PROP_FPS, KAMERA_FPS)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    for _ in range(10):
        ok, frame = cam.read()
        if ok and frame is not None:
            return cam
        time.sleep(0.05)
    cam.release(); print(f'[KAMERA] /dev/video{index} bez obrazu'); return None

def init_camera():
    global camera
    for idx in (CAMERA_SCAN_INDICES if AUTO_CAMERA else [CAMERA_INDEX]):
        try:
            cam = try_open_camera(idx)
            if cam is not None:
                camera = cam
                with state_lock:
                    state['camera_ok'] = True; state['camera_index'] = idx
                print(f'[KAMERA] OK /dev/video{idx}')
                return True
        except Exception as e:
            print(f'[KAMERA] Błąd /dev/video{idx}: {e}')
    with state_lock:
        state['camera_ok'] = False; state['camera_index'] = None; state['status'] = 'Błąd kamery: nie znaleziono /dev/video0..4'
    return False

def read_camera_frame():
    if camera is None: return False, None
    with camera_lock:
        ok, frame = camera.read()
    return (ok and frame is not None), frame

def get_public_state():
    with state_lock:
        return {
            'status': state['status'],
            'nazwa': state['nazwa_obiektu'],
            'czy_sledzic': state['czy_sledzic'],
            'mode': state['mode'],
            'manual_direction': state['manual_direction'],
            'camera_ok': state['camera_ok'],
            'camera_index': state['camera_index'],
            'battery': state['battery'],
            'current_A': state['current_A'],
            'voltage_V': state['voltage_V'],
            'collision_alert': state['collision_alert'],
            'collision_axis': state['collision_axis'],
            'collision_event_id': state['collision_event_id'],
            'gpio_collision': state.get('gpio_collision', False),
            'gpio_collision_pin': state.get('gpio_collision_pin', PIN_KOLIZJI),
            'location_mode': state['location_mode'],
            'calibration_count': len(state['calibration_points']),
            'calibrated': state['calibrated'],
            'calibration_offset_x': state['calibration_offset_x'],
            'calibration_offset_y': state['calibration_offset_y'],
            'calibration_model_ready': state['calibration_model'] is not None or state.get('calibration_altaz_model') is not None,
            'auto_speed_percent': state['auto_speed_percent'],
            'auto_step_chunk': state['auto_step_chunk'],
            'auto_step_delay': state['auto_step_delay'],
            'guide_rate_ready': state.get('guide_rate_ready', False),
            'guide_rate_x': state.get('guide_rate_x', 0.0),
            'guide_rate_y': state.get('guide_rate_y', 0.0),
            'guide_source': state.get('guide_source'),
            'guide_anchor_object': state.get('guide_anchor_object'),
            'goto_pending': state.get('goto_pending', False),
        }

def emit_state(): socketio.emit('update', get_public_state())
def emit_status(msg):
    with state_lock: state['status'] = msg
    emit_state()

def zatrzymaj_silniki():
    try: silnik_x.Stop()
    except Exception: pass
    try: silnik_y.Stop()
    except Exception: pass

def wybierz_kierunek(liczba_krokow, kierunek_plus, kierunek_minus): return kierunek_plus if liczba_krokow > 0 else kierunek_minus

def policz_roznice_ra(ra_start, ra_cel):
    r = ra_cel - ra_start
    if r > 12: r -= 24
    if r < -12: r += 24
    return r

def pobierz_obiekt_ra_dec(obiekt):
    obs = miejsce.at(ts.now()).observe(obiekt).apparent()
    ra, dec, _ = obs.radec()
    return ra.hours, dec.degrees

def pobierz_obiekt_ra_dec_alt_az(obiekt):
    """
    Zwraca jednocześnie RA/DEC i ALT/AZ dla aktualnego czasu.
    Do GoTo bez idealnej Polarnej używamy przede wszystkim ALT/AZ, bo te kąty
    realnie zmieniają się na niebie wraz z obrotem Ziemi.
    """
    obs = miejsce.at(ts.now()).observe(obiekt).apparent()
    ra, dec, _ = obs.radec()
    alt, az, _ = obs.altaz()
    return ra.hours, dec.degrees, alt.degrees, az.degrees

def _az_diff(az_start, az_cel):
    """Najkrótsza różnica azymutu w stopniach, z zawijaniem 0/360."""
    r = az_cel - az_start
    if r > 180:
        r -= 360
    if r < -180:
        r += 360
    return r

def get_object_by_name(name):
    if name in BRIGHT_STARS: return BRIGHT_STARS[name], OBJECT_LABELS.get(name, name), False
    return {'sun':(sun,'Słońce',False),'moon':(moon,'Księżyc',False),'mercury':(eph['mercury'],'Merkury',False),'venus':(eph['venus'],'Wenus',False),'mars':(eph['mars'],'Mars',False),'jupiter':(eph['jupiter barycenter'],'Jowisz',False),'saturn':(eph['saturn barycenter'],'Saturn',False),'uranus':(eph['uranus barycenter'],'Uran',False),'neptune':(eph['neptune barycenter'],'Neptun',False),'iss':(iss,'ISS',True)}.get(name, (None, 'Nieznany', False))

def ustaw_alarm_kolizji(os_nazwa, current_A=None, powod='kolizja'):
    teraz = time.time()
    with state_lock:
        if teraz - state['last_collision_time'] < KOLIZJA_COOLDOWN_S:
            return
        state['collision_alert'] = True
        state['collision_axis'] = os_nazwa
        state['collision_event_id'] += 1
        state['last_collision_time'] = teraz
        state['gpio_collision'] = True if powod == 'gpio' else state.get('gpio_collision', False)
        event_id = state['collision_event_id']
    if current_A is None:
        emit_status(f'KOLIZJA! Czujnik/limit osi {str(os_nazwa).upper()} — zatrzymuję teleskop')
    else:
        emit_status(f'Kolizja / limit osi {str(os_nazwa).upper()} | I={current_A:.3f} A')
    def clear_later(local_event_id):
        time.sleep(CZAS_BANNERA_KOLIZJI_S)
        with state_lock:
            if state['collision_event_id'] == local_event_id:
                state['collision_alert'] = False
                state['collision_axis'] = None
                state['gpio_collision'] = czy_kolizja_gpio(silent=True)
        emit_state()
    threading.Thread(target=clear_later, args=(event_id,), daemon=True).start()

def czy_kolizja_gpio(silent=False):
    if not GPIO_OK or not gpio_kolizja_ok or not KOLIZJA_GPIO_AKTYWNA:
        return False
    try:
        aktywna = GPIO.input(PIN_KOLIZJI) == GPIO.LOW
        with state_lock:
            state['gpio_collision'] = bool(aktywna)
        return bool(aktywna)
    except Exception as e:
        if not silent:
            print(f'Błąd odczytu czujnika kolizji GPIO: {e}')
        return False

def obsluz_kolizje_gpio(os_nazwa='gpio'):
    if czy_kolizja_gpio():
        zatrzymaj_silniki()
        ustaw_alarm_kolizji(os_nazwa, None, powod='gpio')
        return True
    return False

def ruch_awaryjny_bez_testu(silnik, liczba_krokow, os_nazwa, kierunek_plus, kierunek_minus, stepdelay=None):
    if liczba_krokow == 0:
        return
    if stepdelay is None:
        stepdelay = max(opoznienie_kroku, 0.0005)
    kierunek = wybierz_kierunek(liczba_krokow, kierunek_plus, kierunek_minus)
    znak = 1 if liczba_krokow > 0 else -1
    kroki = abs(int(liczba_krokow))
    with motor_lock:
        silnik.TurnStep(Dir=kierunek, steps=kroki, stepdelay=stepdelay)
    with state_lock:
        if os_nazwa == 'x':
            state['aktualne_kroki_x'] += znak * kroki
        else:
            state['aktualne_kroki_y'] += znak * kroki

def awaryjne_ominiecie_kolizji():
    # Krótko cofa i przesuwa trasę w bok. Potem pętla GoTo/śledzenia liczy cel dalej od nowa.
    emit_status('KOLIZJA! Cofam i próbuję ominąć przeszkodę')
    try:
        ruch_awaryjny_bez_testu(silnik_x, -KROKI_COFANIA_KOLIZJI_X, 'x', kierunek_x_plus, kierunek_x_minus, max(opoznienie_kroku, 0.0006))
        ruch_awaryjny_bez_testu(silnik_y, KROKI_OMIJANIA_KOLIZJI_Y, 'y', kierunek_y_plus, kierunek_y_minus, max(opoznienie_kroku, 0.0006))
        time.sleep(CZAS_PO_OMINIECIU_KOLIZJI_S)
        emit_status('Ominięcie wykonane — wracam do śledzenia celu')
    except Exception as e:
        emit_status(f'Błąd awaryjnego ominięcia: {e}')

def wyczysc_alarm_kolizji():
    with state_lock:
        state['collision_alert'] = False
        state['collision_axis'] = None
        state['gpio_collision'] = False

def przelicz_ra_dec_na_kroki_od_polarnej(ra, dec):
    return round(policz_roznice_ra(polarna_ra, ra) * kroki_x_na_1_godzine_ra), round((dec - polarna_dec) * kroki_y_na_1_stopien_dec)

def dodaj_punkt_kalibracyjny(object_name):
    """
    Zapisuje punkt kalibracyjny 3-point alignment.
    Użytkownik najpierw ręcznie centruje obiekt w celowniku kamery,
    a potem klika „Zapisz punkt”. Program zapisuje:
    - obiekt,
    - jego aktualne RA/DEC z Skyfield,
    - aktualne kroki silników X/Y.
    """
    obj, label, _ = get_object_by_name(object_name)
    if obj is None:
        return False, 'Nieznany obiekt kalibracyjny'
    try:
        ra, dec, alt, az = pobierz_obiekt_ra_dec_alt_az(obj)
        czas_str = time.strftime('%H:%M:%S')
        with state_lock:
            state['calibration_points'].append({
                'object': object_name,
                'label': label,
                'ra': ra,
                'dec': dec,
                'alt': alt,
                'az': az,
                'steps_x': state['aktualne_kroki_x'],
                'steps_y': state['aktualne_kroki_y'],
                'time': time.time(),
                'time_str': czas_str
            })
            count = len(state['calibration_points'])
        return True, f'Zapisano punkt {count}/3: {label} {czas_str}. Wycentruj kolejny obiekt i zapisz następny punkt.'
    except Exception as e:
        return False, f'Błąd zapisu punktu kalibracyjnego: {e}'


def _unwrap_ra_for_calibration(ra_ref, ra):
    """Zwraca RA obiektu w godzinach, bez skoku 0/24 h względem punktu referencyjnego."""
    return ra_ref + policz_roznice_ra(ra_ref, ra)


def _fit_affine_3point(points):
    """
    Liczy model afiniczny:
        steps_x = a*RA + b*DEC + c
        steps_y = d*RA + e*DEC + f

    Uwaga: 3x ten sam obiekt w krótkim czasie NIE daje pełnej mapy GoTo,
    bo jego RA/DEC prawie się nie zmienia. Wtedy funkcja celowo odrzuca model
    i kod przechodzi w tryb kotwicy RA/DEC + prowadzenie z prędkości czasowej.
    """
    if len(points) < 3:
        raise ValueError('Potrzeba minimum 3 punktów kalibracyjnych')

    ra_ref = points[0]['ra']
    A = []
    bx = []
    by = []
    ra_vals = []
    dec_vals = []
    for p in points:
        ra_u = _unwrap_ra_for_calibration(ra_ref, float(p['ra']))
        dec = float(p['dec'])
        A.append([ra_u, dec, 1.0])
        bx.append(float(p['steps_x']))
        by.append(float(p['steps_y']))
        ra_vals.append(ra_u)
        dec_vals.append(dec)

    A = np.array(A, dtype=float)
    bx = np.array(bx, dtype=float)
    by = np.array(by, dtype=float)

    rank = np.linalg.matrix_rank(A, tol=1e-9)
    cond = np.linalg.cond(A) if A.shape[0] >= 3 else float('inf')
    spread_ra_deg = (max(ra_vals) - min(ra_vals)) * 15.0
    spread_dec_deg = max(dec_vals) - min(dec_vals)

    # Pełny model GoTo wymaga punktów rozłożonych po niebie.
    # Same Słońce zapisane co kilka minut ma praktycznie ten sam RA/DEC,
    # więc pełny model byłby matematycznie fałszywy.
    if rank < 3 or cond > 10000 or (abs(spread_ra_deg) < 0.2 and abs(spread_dec_deg) < 0.2):
        raise ValueError(
            f'punkty RA/DEC za blisko siebie — pełny model GoTo niestabilny '
            f'(spreadRA={spread_ra_deg:.3f}°, spreadDEC={spread_dec_deg:.3f}°, cond={cond:.1f})'
        )

    coef_x, *_ = np.linalg.lstsq(A, bx, rcond=None)
    coef_y, *_ = np.linalg.lstsq(A, by, rcond=None)

    return ra_ref, {
        'x': [float(v) for v in coef_x],
        'y': [float(v) for v in coef_y],
    }

def _model_ra_dec_to_steps(model, ra_ref, ra, dec):
    ra_u = _unwrap_ra_for_calibration(ra_ref, ra)
    a, b, c = model['x']
    d, e, f = model['y']
    return round(a * ra_u + b * dec + c), round(d * ra_u + e * dec + f)


def _unwrap_az_for_calibration(az_ref, az):
    return az_ref + _az_diff(az_ref, az)

def _fit_affine_altaz(points):
    """
    Liczy model afiniczny lokalnego nieba:
        steps_x = a*AZ + b*ALT + c
        steps_y = d*AZ + e*ALT + f

    To jest ważniejsze niż RA/DEC dla montażu, który nie stoi idealnie na północ,
    bo ALT/AZ pokazuje rzeczywistą pozycję obiektu nad horyzontem w danej chwili.
    """
    if len(points) < 3:
        raise ValueError('Potrzeba minimum 3 punktów kalibracyjnych')
    az_ref = points[0].get('az')
    if az_ref is None:
        raise ValueError('Punkty kalibracji nie mają ALT/AZ')

    A, bx, by = [], [], []
    for p in points:
        az_u = _unwrap_az_for_calibration(az_ref, float(p['az']))
        alt = float(p['alt'])
        A.append([az_u, alt, 1.0])
        bx.append(float(p['steps_x']))
        by.append(float(p['steps_y']))

    A = np.array(A, dtype=float)
    bx = np.array(bx, dtype=float)
    by = np.array(by, dtype=float)

    # Jeżeli punkty są prawie jedną linią, model 2D będzie fałszywy.
    # Wtedy nie udajemy pełnego GoTo, tylko używamy trybu kotwicy + różnic ALT/AZ.
    rank = np.linalg.matrix_rank(A, tol=1e-6)
    spread_az = max([_unwrap_az_for_calibration(az_ref, float(p['az'])) for p in points]) - min([_unwrap_az_for_calibration(az_ref, float(p['az'])) for p in points])
    spread_alt = max(float(p['alt']) for p in points) - min(float(p['alt']) for p in points)
    cond = np.linalg.cond(A) if A.shape[0] >= 3 else float('inf')
    if rank < 3 or cond > 5000 or (abs(spread_az) < 0.05 and abs(spread_alt) < 0.05):
        raise ValueError(f'Punkty są za blisko / prawie w jednej linii — nie ma pełnego modelu GoTo, cond={cond:.1f}')

    coef_x, *_ = np.linalg.lstsq(A, bx, rcond=None)
    coef_y, *_ = np.linalg.lstsq(A, by, rcond=None)
    return az_ref, {'x': [float(v) for v in coef_x], 'y': [float(v) for v in coef_y]}

def _model_altaz_to_steps(model, az_ref, alt, az):
    az_u = _unwrap_az_for_calibration(az_ref, az)
    a, b, c = model['x']
    d, e, f = model['y']
    return round(a * az_u + b * alt + c), round(d * az_u + e * alt + f)

def _fallback_altaz_delta_to_steps(cur_alt, cur_az, target_alt, target_az):
    """
    Awaryjna geometria, gdy wiemy, gdzie teraz patrzy teleskop, ale nie mamy pełnego
    modelu 2D. To sprawia, że po 3x Słońcu teleskop NADAL pojedzie w stronę Jowisza.
    Dokładność zależy od mechaniki i ustawienia osi, ale silniki nie stoją bez sensu.
    """
    dx = round(_az_diff(cur_az, target_az) * (kroki_x_na_360 / 360.0))
    dy = round((target_alt - cur_alt) * (kroki_y_na_360 / 360.0))
    return dx, dy



def _policz_predkosc_prowadzenia_z_punktow(points):
    """
    Liczy prędkość prowadzenia z punktów tego samego obiektu zapisanych w czasie.

    Przykład: Słońce 10:00, 10:05, 10:15.
    Jeżeli użytkownik za każdym razem ręcznie wycentrował Słońce, różnica kroków
    między punktami mówi, z jaką prędkością silniki muszą pracować, żeby obiekt
    nie uciekał z kadru. Wynik to kroki/sekundę dla osi X i Y.
    """
    groups = {}
    for p in points:
        groups.setdefault(p.get('object'), []).append(p)

    best_name = None
    best_group = []
    for name, group in groups.items():
        group = sorted(group, key=lambda x: float(x.get('time', 0)))
        if len(group) > len(best_group):
            best_name = name
            best_group = group

    if len(best_group) < 2:
        return False, 0.0, 0.0, None, 'brak minimum 2 punktów tego samego obiektu'

    rates_x = []
    rates_y = []
    for a, b in zip(best_group[:-1], best_group[1:]):
        dt = float(b['time']) - float(a['time'])
        if dt < 5:
            continue
        rates_x.append((float(b['steps_x']) - float(a['steps_x'])) / dt)
        rates_y.append((float(b['steps_y']) - float(a['steps_y'])) / dt)

    if not rates_x:
        return False, 0.0, 0.0, best_name, 'punkty są za blisko czasowo'

    rx = sum(rates_x) / len(rates_x)
    ry = sum(rates_y) / len(rates_y)
    if abs(rx) < 0.001 and abs(ry) < 0.001:
        return False, rx, ry, best_name, 'wyliczona prędkość ~0; prawdopodobnie punkty zapisano bez ponownego centrowania'

    return True, rx, ry, best_name, f'prowadzenie {best_name}: X={rx:.4f} kr/s, Y={ry:.4f} kr/s'


def _ostatni_punkt_dla_obiektu(points, object_name):
    """Zwraca ostatni zapisany punkt dla danego obiektu, np. ostatnie Słońce."""
    if not object_name:
        return None
    same = [p for p in points if p.get('object') == object_name]
    if not same:
        return None
    return sorted(same, key=lambda x: float(x.get('time', 0)))[-1]


def _predicted_guide_source_steps_now():
    """
    Zwraca przewidywaną aktualną pozycję w krokach dla obiektu użytego do kalibracji czasowej.

    Przykład: kalibracja 3x Słońce zapisuje ostatni punkt Słońca oraz prędkość X/Y
    w krokach na sekundę. Po 10 minutach Słońce nie jest już w starym punkcie, więc
    jego przewidywane kroki to:
        anchor_steps + guide_rate * czas_od_anchor

    Dzięki temu powrót na Słońce nie wraca do starego punktu kalibracji, tylko do
    miejsca, w którym Słońce powinno być teraz.
    """
    with state_lock:
        ready = bool(state.get('guide_rate_ready'))
        source_name = state.get('guide_source')
        anchor_time = state.get('guide_anchor_time')
        anchor_x = state.get('guide_anchor_steps_x')
        anchor_y = state.get('guide_anchor_steps_y')
        rx = float(state.get('guide_rate_x', 0.0))
        ry = float(state.get('guide_rate_y', 0.0))

    if not ready or not source_name or anchor_time is None or anchor_x is None or anchor_y is None:
        return None

    dt = time.time() - float(anchor_time)
    predicted_x = float(anchor_x) + rx * dt
    predicted_y = float(anchor_y) + ry * dt
    return source_name, predicted_x, predicted_y, dt, rx, ry


def _target_steps_from_temporal_anchor(target_obj, target_name):
    """
    Liczy docelowe kroki z kalibracji czasowej, np. 3x Słońce.

    Logika:
    1. Z kalibracji wiemy, gdzie w krokach powinno być TERAZ Słońce / obiekt źródłowy.
    2. Skyfield mówi, gdzie TERAZ jest obiekt źródłowy i gdzie TERAZ jest cel.
    3. Różnica RA/DEC między źródłem a celem zamieniana jest na kroki.

    Jeśli celem jest znowu Słońce, delta RA/DEC wynosi praktycznie 0, więc kod jedzie
    dokładnie do aktualnej, przesuniętej w czasie pozycji Słońca.
    """
    predicted = _predicted_guide_source_steps_now()
    if predicted is None:
        return None

    source_name, source_x, source_y, dt, rx, ry = predicted
    source_obj, source_label, _ = get_object_by_name(source_name)
    if source_obj is None:
        return None

    source_ra, source_dec, _, _ = pobierz_obiekt_ra_dec_alt_az(source_obj)
    target_ra, target_dec, target_alt, target_az = pobierz_obiekt_ra_dec_alt_az(target_obj)

    d_ra_h = policz_roznice_ra(source_ra, target_ra)
    d_dec_deg = target_dec - source_dec

    target_x = round(source_x + d_ra_h * kroki_x_na_1_godzine_ra)
    target_y = round(source_y + d_dec_deg * kroki_y_na_1_stopien_dec)

    opis = (
        f'kotwica czasowa {source_label}: dt={dt:.1f}s, '
        f'predX={source_x:.1f}, predY={source_y:.1f}, '
        f'guide X={rx:.4f}kr/s Y={ry:.4f}kr/s, '
        f'delta do celu RA={d_ra_h:.4f}h DEC={d_dec_deg:.4f}°'
    )
    return target_x, target_y, target_ra, target_dec, target_alt, target_az, opis

def policz_kalibracje_montazu():
    """
    Kończy kalibrację zgodnie z założeniem projektu:

    1. Ostatni wycentrowany punkt mówi: teleskop TERAZ patrzy w konkretny RA/DEC.
       To jest najważniejsza kotwica pozycji.
    2. Jeżeli punkty są rozłożone po niebie, budujemy pełniejszy model RA/DEC -> kroki.
    3. Jeżeli punkty to np. 3x Słońce w odstępach czasu, pełny model GoTo jest niemożliwy,
       ale nadal mamy kotwicę RA/DEC oraz prędkość prowadzenia z różnicy kroków/czasu.
    4. Po tej kalibracji kliknięcie Jowisza/Wenus/Księżyca liczy ruch z aktualnego RA/DEC
       do RA/DEC celu. Czyli silniki mają ruszyć zawsze, o ile cel nie wychodzi poza limity.
    """
    with state_lock:
        points = list(state['calibration_points'])
    if len(points) < 3:
        return False, 'Potrzeba minimum 3 punktów kalibracyjnych. Wycentruj obiekt w celowniku i zapisz minimum 3 punkty.'

    last = points[-1]

    # 1) Próbujemy pełnego modelu RA/DEC -> kroki, ale tylko gdy geometria punktów ma sens.
    ra_ref = None
    radec_model = None
    model_msg = 'kotwica RA/DEC'
    try:
        ra_ref, radec_model = _fit_affine_3point(points)
        model_msg = 'pełny model RA/DEC -> kroki aktywny'
    except Exception as e:
        model_msg = f'kotwica RA/DEC bez pełnego modelu ({e})'

    # 2) Niezależnie liczymy prowadzenie z punktów tego samego obiektu w czasie.
    guide_ok, guide_x, guide_y, guide_source, guide_msg = _policz_predkosc_prowadzenia_z_punktow(points)
    guide_anchor = _ostatni_punkt_dla_obiektu(points, guide_source) if guide_ok else None

    with state_lock:
        state['calibration_model'] = radec_model
        state['calibration_ra_ref'] = ra_ref

        # ALT/AZ zostawiamy tylko informacyjnie / awaryjnie. Główne GoTo jest RA/DEC.
        state['calibration_altaz_model'] = None
        state['calibration_az_ref'] = None

        state['calibrated'] = True
        state['location_mode'] = 'kalibracja ' + model_msg

        # Najważniejsze: po kalibracji wiemy, gdzie aktualnie patrzy teleskop.
        state['aktualne_ra'] = float(last['ra'])
        state['aktualne_dec'] = float(last['dec'])
        state['aktualne_alt'] = float(last.get('alt', 0.0))
        state['aktualne_az'] = float(last.get('az', 0.0))
        state['aktualne_kroki_x'] = int(last['steps_x'])
        state['aktualne_kroki_y'] = int(last['steps_y'])

        state['guide_rate_ready'] = bool(guide_ok)
        state['guide_rate_x'] = float(guide_x) if guide_ok else 0.0
        state['guide_rate_y'] = float(guide_y) if guide_ok else 0.0
        state['guide_source'] = guide_source if guide_ok else None
        state['guide_anchor_object'] = guide_source if guide_ok else None
        state['guide_anchor_time'] = float(guide_anchor['time']) if guide_anchor is not None else None
        state['guide_anchor_steps_x'] = float(guide_anchor['steps_x']) if guide_anchor is not None else None
        state['guide_anchor_steps_y'] = float(guide_anchor['steps_y']) if guide_anchor is not None else None
        state['goto_pending'] = False

        state['calibration_offset_x'] = 0
        state['calibration_offset_y'] = 0

    return True, (
        f'Kalibracja OK | punkty={len(points)} | {model_msg} | '
        f'aktualnie patrzę w: {last["label"]} RA={float(last["ra"]):.4f} DEC={float(last["dec"]):.4f} | '
        f'{guide_msg}'
    )

def wyczysc_kalibracje():
    with state_lock:
        state['calibration_points'] = []
        state['calibrated'] = False
        state['calibration_offset_x'] = 0
        state['calibration_offset_y'] = 0
        state['calibration_model'] = None
        state['calibration_ra_ref'] = None
        state['calibration_altaz_model'] = None
        state['calibration_az_ref'] = None
        state['aktualne_alt'] = None
        state['aktualne_az'] = None
        state['guide_rate_ready'] = False
        state['guide_rate_x'] = 0.0
        state['guide_rate_y'] = 0.0
        state['guide_source'] = None
        state['guide_anchor_object'] = None
        state['guide_anchor_time'] = None
        state['guide_anchor_steps_x'] = None
        state['guide_anchor_steps_y'] = None
        state['obiekt_cel_name'] = None
        state['goto_pending'] = False
        state['location_mode'] = LOCATION_MODE_LABEL

def opis_punktow_kalibracji():
    with state_lock: points = list(state['calibration_points'])
    if not points: return 'Brak punktów kalibracji'
    return ' | '.join([f"{i}. {p['label']} {p['time_str']} ALT={p.get('alt',0):.2f} AZ={p.get('az',0):.2f} X={p['steps_x']} Y={p['steps_y']}" for i,p in enumerate(points[-5:], start=max(1,len(points)-4))])

def ustaw_polarna_jako_start():
    with state_lock:
        state.update({'aktualne_ra':polarna_ra,'aktualne_dec':polarna_dec,'aktualne_kroki_x':0,'aktualne_kroki_y':0,'czy_sledzic':False,'obiekt_cel':None,'obiekt_cel_name':None,'nazwa_obiektu':'Brak','czy_iss':False,'manual_direction':None})
    wyczysc_alarm_kolizji(); zatrzymaj_silniki(); emit_status('Pozycja Polarnej zapisana')

def wroc_na_polarna():
    with state_lock:
        state['czy_sledzic'] = False; state['manual_direction'] = None; ruch_x = -state['aktualne_kroki_x']; ruch_y = -state['aktualne_kroki_y']
    wyczysc_alarm_kolizji(); emit_status('Powrót na Polarną...')
    with motor_lock:
        if ruch_x: silnik_x.TurnStep(Dir=wybierz_kierunek(ruch_x,kierunek_x_plus,kierunek_x_minus), steps=abs(ruch_x), stepdelay=opoznienie_kroku)
        if ruch_y: silnik_y.TurnStep(Dir=wybierz_kierunek(ruch_y,kierunek_y_plus,kierunek_y_minus), steps=abs(ruch_y), stepdelay=opoznienie_kroku)
    with state_lock:
        state.update({'aktualne_kroki_x':0,'aktualne_kroki_y':0,'aktualne_ra':polarna_ra,'aktualne_dec':polarna_dec,'obiekt_cel':None,'obiekt_cel_name':None,'nazwa_obiektu':'Brak','czy_iss':False})
    zatrzymaj_silniki(); emit_status('Teleskop na pozycji Polarnej')

def mapuj_manual_direction(direction): return {'up':(0,reczne_kroki_y),'down':(0,-reczne_kroki_y),'left':(-reczne_kroki_x,0),'right':(reczne_kroki_x,0)}.get(direction,(0,0))

def zapisz_zdjecie():
    ok, frame = read_camera_frame()
    if not ok: return False, 'Nie udało się pobrać klatki z kamery'
    nazwa = f"zdjecie_{time.strftime('%Y%m%d_%H%M%S')}.jpg"; sciezka = os.path.join(folder_zapisu, nazwa)
    return (True, nazwa) if cv2.imwrite(sciezka, frame) else (False, 'cv2.imwrite nie zapisał pliku')

def init_ina219():
    global ina_sensor
    if not ups_sensor_ok: print('UPS/INA219 niedostępny'); return False
    try: ina_sensor = INA219(addr=0x41); print('INA219 uruchomiony'); return True
    except Exception as e: print(f'Błąd inicjalizacji INA219: {e}'); ina_sensor = None; return False

def read_voltage_current():
    if not ups_sensor_ok or ina_sensor is None: return None, None
    try:
        with ina_lock: return ina_sensor.getBusVoltage_V(), ina_sensor.getCurrent_mA()/1000.0
    except Exception as e: print(f'Błąd odczytu INA219: {e}'); return None, None

def czy_przeciazenie_potwierdzone():
    trafienia = 0; ostatni = None
    for _ in range(ILOSC_PROBEK_KOLIZJI):
        _, current_A = read_voltage_current()
        if current_A is None: time.sleep(ODSTEP_PROBEK_KOLIZJI); continue
        ostatni = current_A
        if POKAZUJ_DIAGNOSTYKE_KOLIZJI: print(f'[KOLIZJA TEST] current_A={current_A:.4f} A')
        if current_A <= PROG_KOLIZJI_A: trafienia += 1
        time.sleep(ODSTEP_PROBEK_KOLIZJI)
    return trafienia >= MIN_TRAFIEN_DO_KOLIZJI, ostatni

def battery_worker():
    if not ups_sensor_ok: print('UPS/INA219 niedostępny'); return
    while True:
        v, a = read_voltage_current()
        if v is not None and a is not None:
            p = max(0, min(100, (v-9)/3.6*100))
            with state_lock: state['battery']=round(p,1); state['current_A']=round(a,4); state['voltage_V']=round(v,3)
            emit_state()
        time.sleep(2)

def rusz_os_paczka(silnik, liczba_krokow, os_nazwa, kierunek_plus, kierunek_minus, stepdelay=None):
    """
    Ruch osi bez ograniczania po rosnącym prądzie.
    INA219 dalej pokazuje napięcie/prąd/baterię, ale NIE zatrzymuje silników.
    """
    if liczba_krokow == 0:
        return True, None

    if obsluz_kolizje_gpio(os_nazwa):
        return False, None

    if stepdelay is None:
        stepdelay = opoznienie_kroku

    kierunek = wybierz_kierunek(liczba_krokow, kierunek_plus, kierunek_minus)
    znak = 1 if liczba_krokow > 0 else -1
    kroki = abs(liczba_krokow)

    with motor_lock:
        silnik.TurnStep(
            Dir=kierunek,
            steps=kroki,
            stepdelay=stepdelay
        )

    with state_lock:
        if os_nazwa == "x":
            state["aktualne_kroki_x"] += znak * kroki
        else:
            state["aktualne_kroki_y"] += znak * kroki

    return True, None

def wykonaj_ruch_reczny(dx, dy):
    with state_lock:
        if state['czy_sledzic']: return False, 'Ręczne sterowanie zablokowane podczas śledzenia'
        if state['mode'] != 'manual': return False, 'Nie jesteś w trybie ręcznym'
        delay = state['manual_step_delay']; x = state['aktualne_kroki_x']; y = state['aktualne_kroki_y']
    if dx:
        if not (min_x <= x+dx <= max_x): ustaw_alarm_kolizji('x', state.get('current_A',0.0)); return False, 'Limit X'
        ok, cur = rusz_os_paczka(silnik_x, dx, 'x', kierunek_x_plus, kierunek_x_minus, delay)
        if not ok: return False, f'Kolizja osi X | I={(cur or 0.0):.3f} A'
    if dy:
        if not (min_y <= y+dy <= max_y): ustaw_alarm_kolizji('y', state.get('current_A',0.0)); return False, 'Limit Y'
        ok, cur = rusz_os_paczka(silnik_y, dy, 'y', kierunek_y_plus, kierunek_y_minus, delay)
        if not ok: return False, f'Kolizja osi Y | I={(cur or 0.0):.3f} A'

    # Bardzo ważne: po ręcznym przesunięciu teleskop NIE patrzy już w poprzednie RA/DEC.
    # Bez tego po kalibracji na Słońcu można było przesunąć teleskop w bok,
    # a kod nadal myślał, że jest na Słońcu, więc przy ponownym śledzeniu Słońca
    # wyliczał ruch ~0 i nie wracał.
    with state_lock:
        state['aktualne_ra'] = (state['aktualne_ra'] + (dx / kroki_x_na_1_godzine_ra if dx else 0.0)) % 24.0
        state['aktualne_dec'] = max(-90.0, min(90.0, state['aktualne_dec'] + (dy / kroki_y_na_1_stopien_dec if dy else 0.0)))
    return True, None

def _wykonaj_ruch_do_delta(kx, ky, nazwa, opis_celu, auto_delay, auto_chunk):
    with state_lock:
        ax = state['aktualne_kroki_x']
        ay = state['aktualne_kroki_y']

    if abs(kx) < 1 and abs(ky) < 1:
        emit_status(f'{nazwa}: pozycja bez zmian / ruch < 1 krok | {opis_celu}')
        return True

    if not (min_x <= ax + kx <= max_x):
        with state_lock:
            state['czy_sledzic'] = False
            state['obiekt_cel'] = None
        zatrzymaj_silniki()
        emit_status(f'STOP: limit osi X | planowany ruch X={kx}')
        return False
    if not (min_y <= ay + ky <= max_y):
        with state_lock:
            state['czy_sledzic'] = False
            state['obiekt_cel'] = None
        zatrzymaj_silniki()
        emit_status(f'STOP: limit osi Y | planowany ruch Y={ky}')
        return False

    emit_status(f'Jadę/śledzę: {nazwa} | ruch X={kx} Y={ky} | {opis_celu}')
    px, py = int(kx), int(ky)
    while px or py:
        if obsluz_kolizje_gpio('auto'):
            awaryjne_ominiecie_kolizji()
            return False
        with state_lock:
            if not state['czy_sledzic']:
                zatrzymaj_silniki()
                return False
            auto_delay = state.get('auto_step_delay', auto_delay)
            auto_chunk = max(1, int(state.get('auto_step_chunk', auto_chunk)))

        if px:
            step = min(auto_chunk, abs(px))
            step = step if px > 0 else -step
            ok, cur = rusz_os_paczka(silnik_x, step, 'x', kierunek_x_plus, kierunek_x_minus, auto_delay)
            if not ok:
                with state_lock:
                    state.update({'czy_sledzic': False, 'obiekt_cel': None, 'czy_iss': False, 'nazwa_obiektu': 'Brak'})
                zatrzymaj_silniki()
                emit_status(f'STOP: kolizja osi X | I={(cur or 0.0):.3f} A')
                return False
            px -= step

        if py:
            step = min(auto_chunk, abs(py))
            step = step if py > 0 else -step
            ok, cur = rusz_os_paczka(silnik_y, step, 'y', kierunek_y_plus, kierunek_y_minus, auto_delay)
            if not ok:
                with state_lock:
                    state.update({'czy_sledzic': False, 'obiekt_cel': None, 'czy_iss': False, 'nazwa_obiektu': 'Brak'})
                zatrzymaj_silniki()
                emit_status(f'STOP: kolizja osi Y | I={(cur or 0.0):.3f} A')
                return False
            py -= step
    zatrzymaj_silniki()
    return True


def idz_do_obiektu(obj, nazwa):
    """
    Główne GoTo po kalibracji.

    Najważniejsza poprawka: przy kalibracji czasowej, np. 3x Słońce, kod NIE wraca
    do starego punktu kalibracji. Najpierw przewiduje, gdzie obiekt kalibracyjny
    powinien być TERAZ w krokach:
        ostatni_punkt + guide_rate * upływ_czasu
    Dopiero od tej aktualnej kotwicy liczy przejście do celu, np. Jowisza.
    """
    cel_ra, cel_dec, cel_alt, cel_az = pobierz_obiekt_ra_dec_alt_az(obj)
    with state_lock:
        ara = float(state['aktualne_ra'])
        adec = float(state['aktualne_dec'])
        ax = int(state['aktualne_kroki_x'])
        ay = int(state['aktualne_kroki_y'])
        cal = bool(state['calibrated'])
        radec_model = state.get('calibration_model')
        ra_ref = state.get('calibration_ra_ref')
        guide_ready = bool(state.get('guide_rate_ready'))
        auto_delay = state.get('auto_step_delay', opoznienie_kroku)
        auto_chunk = max(1, int(state.get('auto_step_chunk', wielkosc_paczki_auto)))

    # 1) Najpierw obsługujemy kalibrację czasową, np. 3x Słońce.
    # To naprawia błąd: powrót na Słońce po czasie wracał do starego punktu,
    # zamiast do aktualnej pozycji przesuniętej przez ruch nieba.
    temporal_target = _target_steps_from_temporal_anchor(obj, nazwa) if guide_ready else None
    if temporal_target is not None and (radec_model is None or not cal):
        target_x, target_y, cel_ra, cel_dec, cel_alt, cel_az, opis = temporal_target
        kx = target_x - ax
        ky = target_y - ay
        opis = f'RA={cel_ra:.4f} DEC={cel_dec:.4f} | {opis}'

    elif cal and radec_model is not None and ra_ref is not None:
        target_x, target_y = _model_ra_dec_to_steps(radec_model, ra_ref, cel_ra, cel_dec)
        kx = target_x - ax
        ky = target_y - ay
        opis = f'RA={cel_ra:.4f} DEC={cel_dec:.4f} | pełny model RA/DEC'

    else:
        # Awaryjnie: wiem, gdzie jestem w RA/DEC, wiem gdzie jest cel,
        # więc jadę o różnicę RA i DEC.
        d_ra_h = policz_roznice_ra(ara, cel_ra)
        d_dec_deg = cel_dec - adec
        kx = round(d_ra_h * kroki_x_na_1_godzine_ra)
        ky = round(d_dec_deg * kroki_y_na_1_stopien_dec)
        opis = f'RA={cel_ra:.4f} DEC={cel_dec:.4f} | delta RA={d_ra_h:.4f}h DEC={d_dec_deg:.4f}° | kotwica RA/DEC'

    ok = _wykonaj_ruch_do_delta(kx, ky, nazwa, opis, auto_delay, auto_chunk)
    if ok:
        with state_lock:
            state['aktualne_ra'] = cel_ra
            state['aktualne_dec'] = cel_dec
            state['aktualne_alt'] = cel_alt
            state['aktualne_az'] = cel_az
            state['goto_pending'] = False


def idz_do_ra_dec(cel_ra, cel_dec, nazwa):
    # Zostawione tylko dla kompatybilności ze starym kodem.
    with state_lock:
        ara = state['aktualne_ra']
        adec = state['aktualne_dec']
        auto_delay = state.get('auto_step_delay', opoznienie_kroku)
        auto_chunk = max(1, int(state.get('auto_step_chunk', wielkosc_paczki_auto)))
    kx = round(policz_roznice_ra(ara, cel_ra) * kroki_x_na_1_godzine_ra)
    ky = round((cel_dec - adec) * kroki_y_na_1_stopien_dec)
    ok = _wykonaj_ruch_do_delta(kx, ky, nazwa, f'RA={cel_ra:.3f} DEC={cel_dec:.3f}', auto_delay, auto_chunk)
    if ok:
        with state_lock:
            state['aktualne_ra'] = cel_ra
            state['aktualne_dec'] = cel_dec

def rysuj_celownik(frame):
    """Dodaje celownik na środku obrazu z kamery, przydatny podczas kalibracji."""
    try:
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx - 35, cy), (cx + 35, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 35), (cx, cy + 35), (0, 255, 0), 1)
        cv2.circle(frame, (cx, cy), 18, (0, 255, 0), 1)
    except Exception:
        pass
    return frame


def move_x(direction):
    """Korekta osi X dla OpenCV tracking. direction: -1 albo 1."""
    steps = int(reczne_kroki_x)
    
    with state_lock:
        delay = state.get('manual_step_delay', 0.00015)
    return rusz_os_paczka(silnik_x, int(direction) * steps, 'x', kierunek_x_plus, kierunek_x_minus, delay)


def move_y(direction):
    """Korekta osi Y dla OpenCV tracking. direction: -1 albo 1."""
    steps = int(reczne_kroki_y)
    
    with state_lock:
        delay = state.get('manual_step_delay', 0.00015)
    return rusz_os_paczka(silnik_y, int(direction) * steps, 'y', kierunek_y_plus, kierunek_y_minus, delay)

def generate_frames():
    while True:
        ok, frame = read_camera_frame()
        if not ok: frame = make_placeholder_frame('Sprawdz kabel, zasilanie i /dev/video')
        frame = rysuj_celownik(frame)
        ok_jpg, buffer = cv2.imencode('.jpg', frame)
        if ok_jpg: yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n'
        time.sleep(0.03)

def telescope_worker():
    while True:
        with state_lock:
            sledz = state['czy_sledzic']
            obj = state['obiekt_cel']
            nazwa = state['nazwa_obiektu']
            is_iss = state['czy_iss']
            goto_pending = state.get('goto_pending', False)
            guide_ready = state.get('guide_rate_ready', False)

        if sledz and obj is not None:
            try:
                # Po kliknięciu obiektu robimy jeden GoTo do celu.
                # Potem, jeśli mamy guide_rate z kalibracji czasowej, ciągłe prowadzenie robi guide_worker.
                if goto_pending or is_iss or not guide_ready:
                    idz_do_obiektu(obj, nazwa)
                else:
                    # Przy aktywnym prowadzeniu nie przeliczamy co sekundę dużego GoTo,
                    # tylko aktualizujemy zapis RA/DEC celu, a silniki stale kręci guide_worker.
                    ra, dec, alt, az = pobierz_obiekt_ra_dec_alt_az(obj)
                    with state_lock:
                        state['aktualne_ra'] = ra
                        state['aktualne_dec'] = dec
                        state['aktualne_alt'] = alt
                        state['aktualne_az'] = az
            except Exception as e:
                with state_lock:
                    state['czy_sledzic'] = False
                    state['obiekt_cel'] = None
                    state['obiekt_cel_name'] = None
                    state['goto_pending'] = False
                zatrzymaj_silniki()
                emit_status(f'Błąd śledzenia: {e}')
            time.sleep(czas_odswiezania_iss if is_iss else czas_odswiezania_planety)
        else:
            time.sleep(0.2)


def guide_worker():
    """
    Ciągłe prowadzenie w tle z prędkości wyliczonej z kalibracji czasowej.
    Np. po 3x Słońce kod zna, ile kroków/s trzeba robić na X/Y,
    żeby kompensować obrót Ziemi i błąd ustawienia północy.
    """
    carry_x = 0.0
    carry_y = 0.0
    last_t = time.time()
    while True:
        now = time.time()
        dt = now - last_t
        last_t = now

        with state_lock:
            active = bool(state.get('czy_sledzic')) and bool(state.get('guide_rate_ready')) and not bool(state.get('goto_pending'))
            rx = float(state.get('guide_rate_x', 0.0))
            ry = float(state.get('guide_rate_y', 0.0))
            delay = state.get('auto_step_delay', opoznienie_kroku)

        if not active:
            carry_x = 0.0
            carry_y = 0.0
            time.sleep(0.05)
            continue

        carry_x += rx * dt
        carry_y += ry * dt
        sx = int(carry_x)
        sy = int(carry_y)

        if sx != 0:
            carry_x -= sx
            rusz_os_paczka(silnik_x, sx, 'x', kierunek_x_plus, kierunek_x_minus, delay)
        if sy != 0:
            carry_y -= sy
            rusz_os_paczka(silnik_y, sy, 'y', kierunek_y_plus, kierunek_y_minus, delay)

        time.sleep(0.05)


def manual_worker():
    last = 0.0
    while True:
        with state_lock: mode=state['mode']; sledz=state['czy_sledzic']; direction=state['manual_direction']
        if mode == 'manual' and not sledz and direction:
            dx, dy = mapuj_manual_direction(direction); ok, err = wykonaj_ruch_reczny(dx, dy)
            if not ok:
                with state_lock: state['manual_direction'] = None
                zatrzymaj_silniki(); emit_status(err); time.sleep(0.05); continue
            now = time.time()
            if now-last > 0.4: emit_status(f'Ruch ręczny: {direction}'); last = now
            time.sleep(manual_loop_delay)
        else: time.sleep(0.01)

@socketio.on('connect')
def on_connect(): socketio.emit('update', get_public_state())

@socketio.on('command')
def handle_command(data):
    cmd = data.get('action','')
    if cmd == 'set_speed':
        val = max(1, min(100, int(data.get('value', 50))))
        with state_lock:
            state['manual_step_delay'] = 0.0015 * (1 - (val / 100.0)) + 0.00005
        return
    if cmd == 'set_auto_speed':
        val = max(1, min(100, int(data.get('value', 50))))
        # Większy procent = mniejszy delay i większa paczka kroków, czyli szybszy GoTo.
        with state_lock:
            state['auto_speed_percent'] = val
            state['auto_step_delay'] = 0.0012 * (1 - (val / 100.0)) + 0.00003
            state['auto_step_chunk'] = int(50 + (val / 100.0) * 950)
        emit_status(f'Prędkość automatyczna: {val}%')
        return
    if cmd == 'take_photo':
        ok, wynik = zapisz_zdjecie(); emit_status(f'Zdjęcie zapisane: {wynik}' if ok else f'Błąd zapisu zdjęcia: {wynik}'); return
    if cmd == 'calibration_point':
        ok, msg = dodaj_punkt_kalibracyjny(data.get('object','')); emit_status(msg); return
    if cmd == 'finish_calibration':
        ok, msg = policz_kalibracje_montazu(); emit_status(msg); return
    if cmd == 'clear_calibration': wyczysc_kalibracje(); emit_status('Kalibracja wyczyszczona'); return
    if cmd == 'list_calibration': emit_status(opis_punktow_kalibracji()); return
    if cmd == 'stop':
        with state_lock: state.update({'czy_sledzic':False,'obiekt_cel':None,'obiekt_cel_name':None,'czy_iss':False,'manual_direction':None,'nazwa_obiektu':'Brak','goto_pending':False})
        wyczysc_alarm_kolizji(); zatrzymaj_silniki(); emit_status('Zatrzymano wszystko'); return
    if cmd == 'set_home': ustaw_polarna_jako_start(); return
    if cmd == 'go_polar': threading.Thread(target=wroc_na_polarna, daemon=True).start(); return
    if cmd == 'mode_auto':
        with state_lock: state['mode']='auto'; state['manual_direction']=None
        wyczysc_alarm_kolizji(); zatrzymaj_silniki(); emit_status('Tryb automatyczny'); return
    if cmd == 'mode_manual':
        with state_lock:
            if state['czy_sledzic']: emit_status('Nie można wejść w tryb ręczny podczas śledzenia'); return
            state['mode']='manual'; state['manual_direction']=None
        wyczysc_alarm_kolizji(); zatrzymaj_silniki(); emit_status('Tryb ręczny'); return
    if cmd.startswith('manual_start_'):
        direction = cmd.replace('manual_start_','')
        with state_lock:
            if state['czy_sledzic']: emit_status('Nie można sterować ręcznie podczas śledzenia'); return
            if state['mode'] != 'manual': emit_status('Najpierw przełącz na tryb ręczny'); return
            if direction not in ('up','down','left','right'): emit_status('Nieznany kierunek ręczny'); return
            state['manual_direction'] = direction
        emit_status(f'Start ruchu ręcznego: {direction}'); return
    if cmd == 'manual_stop':
        with state_lock: state['manual_direction'] = None
        zatrzymaj_silniki(); emit_status('Zatrzymano ruch ręczny'); return
    shortcuts = {'mars':'mars','jupiter':'jupiter','saturn':'saturn','venus':'venus','moon':'moon','iss':'iss'}
    if cmd == 'track_object' or cmd in shortcuts:
        obj_name = data.get('object','') if cmd == 'track_object' else shortcuts[cmd]
        obj, nazwa, is_iss = get_object_by_name(obj_name)
        if obj is None: emit_status('Nieznany obiekt'); return
        with state_lock:
            if state['mode'] != 'auto': emit_status('Najpierw przełącz na tryb automatyczny'); return
            state.update({'obiekt_cel':obj,'obiekt_cel_name':obj_name,'nazwa_obiektu':nazwa,'czy_iss':is_iss,'czy_sledzic':True,'manual_direction':None,'goto_pending':True})
        wyczysc_alarm_kolizji(); zatrzymaj_silniki(); emit_status(f'Start śledzenia: {nazwa}'); return
    emit_status(f'Nieznana komenda: {cmd}')



def get_saved_photos():
    photos = []
    allowed = ('.jpg', '.jpeg', '.png', '.webp')
    try:
        for name in os.listdir(folder_zapisu):
            if not name.lower().endswith(allowed):
                continue
            path = os.path.join(folder_zapisu, name)
            if not os.path.isfile(path):
                continue
            photos.append({
                'name': name,
                'url': '/photo/' + name,
                'mtime': os.path.getmtime(path),
            })
    except Exception as e:
        print(f'[GALERIA] Błąd odczytu folderu zdjęć: {e}')
    photos.sort(key=lambda p: p['mtime'], reverse=True)
    return photos

@app.route('/gallery')
def gallery():
    return render_template('gallery.html', photos=get_saved_photos(), folder_zapisu=folder_zapisu)

@app.route('/photo/<path:filename>')
def photo_file(filename):
    return send_from_directory(folder_zapisu, filename)

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed(): return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/astro_gps/add', methods=['POST'])
def astro_gps_add():
    data = request.json

    dodaj_obserwacje_astro_gps(
        data['object'],
        float(data['alt']),
        float(data['az'])
    )

    return jsonify({
        "ok": True,
        "count": len(astro_gps_observations)
    })

@app.route('/astro_gps/solve', methods=['GET'])
def astro_gps_solve():
    ok, result = policz_lokalizacje_z_gwiazd()

    return jsonify({
        "ok": ok,
        "result": result
    })



@app.route('/astro_gps/apply', methods=['POST'])
def astro_gps_apply():

    global szerokosc
    global dlugosc
    global miejsce

    data = request.json

    szerokosc = float(data['latitude'])
    dlugosc = float(data['longitude'])

    miejsce = earth + wgs84.latlon(
        szerokosc,
        dlugosc,
        elevation_m=wysokosc_m
    )

    with state_lock:
        state['location_mode'] = f'Astro GPS: {szerokosc:.3f}, {dlugosc:.3f}'
    emit_state()

    return jsonify({
        "ok": True,
        "latitude": szerokosc,
        "longitude": dlugosc
    })





# ============================================================
# OPENCV TRACKING API
# ============================================================

@app.route('/vision_tracking/start', methods=['POST'])
def vision_tracking_start():

    ok = start_vision_tracking()

    return jsonify({
        "ok": ok,
        "tracking": vision_tracking_enabled
    })

@app.route('/vision_tracking/stop', methods=['POST'])
def vision_tracking_stop():

    stop_vision_tracking()

    return jsonify({
        "ok": True,
        "tracking": False
    })

@app.route('/vision_tracking/status', methods=['GET'])
def vision_tracking_status():

    return jsonify({
        "enabled": vision_tracking_enabled,
        "state": vision_tracking_state
    })


if __name__ == '__main__':
    init_camera(); init_ina219()
    threading.Thread(target=telescope_worker, daemon=True).start()
    threading.Thread(target=guide_worker, daemon=True).start()
    threading.Thread(target=manual_worker, daemon=True).start()
    threading.Thread(target=battery_worker, daemon=True).start()
    print('Start serwera teleskopu...')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

