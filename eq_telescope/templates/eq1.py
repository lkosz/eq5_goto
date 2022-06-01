#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time, sys, os, threading, ujson as json, queue, numpy, pigpio, subprocess, board, busio
from digitalio import Direction
from adafruit_mcp230xx.mcp23017 import MCP23017
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from http.server import BaseHTTPRequestHandler, HTTPServer
from astropy import units as u
from astropy.coordinates import SkyCoord,EarthLocation,AltAz,Angle,ITRS
from astropy.time import Time


pi = pigpio.pi()

t_ra = Angle('00h00m00s')
t_dec = Angle('00d00m00s')
t_loc = EarthLocation(lat=51.0982246264821*u.deg, lon=17.019650955565243*u.deg, height=134*u.m)
after_meridian = False

cmd = queue.Queue()
cmd_ra_r = queue.Queue()
cmd_dec_u = queue.Queue()
cmd_ost_l = queue.Queue()
cmd_ra_l = queue.Queue()
cmd_dec_d = queue.Queue()
cmd_ost_r = queue.Queue()
cmd_update_ra = queue.Queue()
cmd_update_dec = queue.Queue()

ra_natural_running = False
f_ra_move_speed = 0
f_dec_move_speed = 0
f_ost_move_speed = 0
ra_natural_speed = 'OFF'
goto_working = False
last_ra_change = 0
last_dec_change = 0
last_radec_change_t = 0
guider_correction_ra = 0
guider_correction_dec = 0

p = {
  'ra_en'   : 4,
  'ra_step' : 10,
  'ra_dir'  : 9,
  'ra_ms0'  : 17,
  'ra_ms1'  : 27,
  'ra_ms2'  : 22,

  'dec_en'   : 11,
  'dec_step' : 19,
  'dec_dir'  : 26,
  'dec_ms0'  : 5,
  'dec_ms1'  : 6,
  'dec_ms2'  : 13,

  'ost_en'   : 18,
  'ost_step' : 8,
  'ost_dir'  : 7,
  'ost_ms0'  : 23,
  'ost_ms1'  : 24,
  'ost_ms2'  : 25,
}

i2c = 0
mcp = 0
ads1 = 0
ads2 = 0
i1 = 0
i2 = 0
i3 = 0
i4 = 0
v1 = 0
v2 = 0
v3 = 0
v4 = 0
i1_real = 0
i2_real = 0
i3_real = 0
i4_real = 0
v1_real = 0
v2_real = 0
v3_real = 0
v4_real = 0
mah1 = 0
mah2 = 0
mah3 = 0
mah4 = 0
b1_state = 'null'
b2_state = 'null'
b3_state = 'null'
b4_state = 'null'

force_on_b1 = False
force_on_b2 = False
force_on_b3 = False
force_on_b4 = False
force_off_b1 = False
force_off_b2 = False
force_off_b3 = False
force_off_b4 = False

ra_star = 86164.091
ra_sun  = 86400
ra_moon = 89428

motor_steps    = 400
mount_ra_gear  = 144
mount_dec_gear = 144

ra_driver_resolution  = 256 * 60 / 16
dec_driver_resolution = 256 * 40 / 16

ra_num_of_steps  = float(motor_steps * mount_ra_gear * ra_driver_resolution)
dec_num_of_steps = float(motor_steps * mount_dec_gear * dec_driver_resolution)

step_delay_star = float(1)/(ra_num_of_steps / float(ra_star))
step_delay_sun  = float(1)/(ra_num_of_steps / float(ra_sun))
step_delay_moon = float(1)/(ra_num_of_steps / float(ra_moon))

dec_angles = {
  'slow':    Angle(1.0/(float(dec_num_of_steps)  / 360.0) * u.deg),
  'fast':    Angle(1.0/(float(dec_num_of_steps) / 4.0 / 360.0) * u.deg),
  'fastest': Angle(1.0/(float(dec_num_of_steps) / 256.0 / 360.0) * u.deg),
}
ra_angles = {
  'slow':    Angle(1.0/(float(ra_num_of_steps)  / 360.0) * u.deg),
  'fast':    Angle(1.0/(float(ra_num_of_steps) / 4.0 / 360.0) * u.deg),
  'fastest': Angle(1.0/(float(ra_num_of_steps) / 256.0 / 360.0) * u.deg),
}


def f_setup():
  global p
  global i2c, mcp, ads1, ads2, i1, i2, i3, i4, v1, v2, v3, v4

  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)

  pi.wave_tx_stop()
  pi.wave_clear()

  for a in p.keys():
    GPIO.setup(p[a], GPIO.OUT)
    GPIO.output(p[a], 0)

  i2c = busio.I2C(board.SCL, board.SDA)
  mcp = MCP23017(i2c, address=0x20)
  ads1 = ADS.ADS1015(i2c, address=0x48)
  ads2 = ADS.ADS1015(i2c, address=0x49)

  i1 = AnalogIn(ads1, ADS.P0)
  i2 = AnalogIn(ads1, ADS.P1)
  i3 = AnalogIn(ads1, ADS.P2)
  i4 = AnalogIn(ads1, ADS.P3)

  v1 = AnalogIn(ads2, ADS.P0)
  v2 = AnalogIn(ads2, ADS.P1)
  v3 = AnalogIn(ads2, ADS.P2)
  v4 = AnalogIn(ads2, ADS.P3)

  for i in range(16):
    pin = mcp.get_pin(i)
    pin.direction = Direction.OUTPUT
    pin.value = False
  f_ra_on()


#----------------------------------------------------

def f_read_battery():
  global i1, i2, i3, i4, v1, v2, v3, v4
  global i1_real, i2_real, i3_real, i4_real, v1_real, v2_real, v3_real, v4_real

  try:
    i1_real = round(abs(i1.voltage - 2.464)/0.066, 2)
    i2_real = round(abs(i2.voltage - 2.495)/0.14, 2)
    i3_real = round(abs(i3.voltage - 2.489)/0.076, 2)
    i4_real = round(abs(i4.voltage - 2.507)/0.082, 2)
    v1_real = round(abs(v1.voltage*3.94), 2)
    v2_real = round(abs(v2.voltage*3.94), 2)
    v3_real = round(abs(v3.voltage*3.94), 2)
    v4_real = round(abs(v4.voltage*3.94), 2)

    if i1_real < 0.19:
      i1_real = 0.0
    if i2_real < 0.19:
      i2_real = 0.0
    if i3_real < 0.19:
      i3_real = 0.0
    if i4_real < 0.19:
      i4_real = 0.0

    if v1_real < 0.9:
      v1_real = 0.0
    if v2_real < 0.9:
      v2_real = 0.0
    if v3_real < 0.9:
      v3_real = 0.0
    if v4_real < 0.9:
      v4_real = 0.0
  except:
    pass

def f_led_on():
  global mcp
  pin = mcp.get_pin(4)
  pin.value = True

def f_led_off():
  global mcp
  pin = mcp.get_pin(4)
  pin.value = False

def f_battery1_on():
  global mcp
  pin = mcp.get_pin(0)
  pin.value = True

def f_battery1_off():
  global mcp
  pin = mcp.get_pin(0)
  pin.value = False

def f_battery2_on():
  global mcp
  pin = mcp.get_pin(1)
  pin.value = True

def f_battery2_off():
  global mcp
  pin = mcp.get_pin(1)
  pin.value = False

def f_battery3_on():
  global mcp
  pin = mcp.get_pin(2)
  pin.value = True

def f_battery3_off():
  global mcp
  pin = mcp.get_pin(2)
  pin.value = False

def f_battery4_on():
  global mcp
  pin = mcp.get_pin(3)
  pin.value = True

def f_battery4_off():
  global mcp
  pin = mcp.get_pin(3)
  pin.value = False

def f_battery():
  global i1_real, i2_real, i3_real, i4_real, v1_real, v2_real, v3_real, v4_real
  global force_on_b1, force_on_b2, force_on_b3, force_on_b4
  global force_off_b1, force_off_b2, force_off_b3, force_off_b4
  global mah1, mah2, mah3, mah4
  global b1_state, b2_state, b3_state, b4_state

  licznik = 0
  last_time = time.time()
  curr_time = time.time()
  while True:
    try:
      f_read_battery()
    except:
      pass
    batt_num = 0

    last_time = curr_time
    curr_time = time.time()
    time_delta = curr_time - last_time
    mah1 = float(mah1) + float(i1_real) * 1000.0 / 3600.0 * float(time_delta)
    mah2 = float(mah2) + float(i2_real) * 1000.0 / 3600.0 * float(time_delta)
    mah3 = float(mah3) + float(i3_real) * 1000.0 / 3600.0 * float(time_delta)
    mah4 = float(mah4) + float(i4_real) * 1000.0 / 3600.0 * float(time_delta)

    if force_off_b1 == True:
      f_battery1_off()
      b1_state = 'FORCE OFF'
    elif force_on_b1 == True:
      batt_num = batt_num + 1
      f_battery1_on()
      b1_state = 'FORCE ON'
    elif (v1_real > 10.1 and v1_real < 15.0):
      batt_num = batt_num + 1
      f_battery1_on()
      b1_state = 'AUTO ON'
    else:
      f_battery1_off()
      b1_state = 'AUTO OFF'

    if force_off_b2 == True:
      f_battery2_off()
      b2_state = 'FORCE OFF'
    elif force_on_b2 == True:
      batt_num = batt_num + 1
      f_battery2_on()
      b2_state = 'FORCE ON'
    elif (v2_real > 10.1 and v2_real < 15.0):
      batt_num = batt_num + 1
      f_battery2_on()
      b2_state = 'AUTO ON'
    else:
      f_battery2_off()
      b2_state = 'AUTO OFF'

    if force_off_b3 == True:
      f_battery3_off()
      b3_state = 'FORCE OFF'
    elif force_on_b3 == True:
      batt_num = batt_num + 1
      f_battery3_on()
      b3_state = 'FORCE ON'
    elif (v3_real > 10.1 and v3_real < 15.0):
      batt_num = batt_num + 1
      f_battery3_on()
      b3_state = 'AUTO ON'
    else:
      f_battery3_off()
      b3_state = 'AUTO OFF'

    if force_off_b4 == True:
      f_battery4_off()
      b4_state = 'FORCE OFF'
    elif force_on_b4 == True:
      batt_num = batt_num + 1
      f_battery4_on()
      b4_state = 'FORCE ON'
    elif (v4_real > 10.1 and v4_real < 15.0):
      batt_num = batt_num + 1
      f_battery4_on()
      b4_state = 'AUTO ON'
    else:
      f_battery4_off()
      b4_state = 'AUTO OFF'


    if batt_num < 1 or v1_real >= 15.0 or v2_real >= 15.0 or v3_real >= 15.0 or v4_real >= 15.0:
      licznik = licznik + 1
      if licznik in [3,4]:
        f_led_on()
        licznik = 1
      elif licznik in [1,2]:
        f_led_off()


    time.sleep(0.5)


#----------------------------------------------------

def f_ra_on():
  global p
  GPIO.output(p['ra_en'], 1)

def f_ra_off():
  global p
  GPIO.output(p['ra_en'], 0)

def f_ra_fastest():
  global p
  GPIO.output(p['ra_ms0'], 0)
  GPIO.output(p['ra_ms1'], 0)
  GPIO.output(p['ra_ms2'], 0)

def f_ra_fast():
  global p
  GPIO.output(p['ra_ms0'], 0)
  GPIO.output(p['ra_ms1'], 1)
  GPIO.output(p['ra_ms2'], 0)

def f_ra_slow():
  global p
  GPIO.output(p['ra_ms0'], 1)
  GPIO.output(p['ra_ms1'], 1)
  GPIO.output(p['ra_ms2'], 1)

def f_ra_left():
  global p
  GPIO.output(p['ra_dir'], 0)

def f_ra_right():
  global p
  GPIO.output(p['ra_dir'], 1)

def f_ra_step():
  global p
  f_ra_on()
  GPIO.output(p['ra_step'], 1)
  time.sleep(0.00000001)
  GPIO.output(p['ra_step'], 0)
  time.sleep(0.00000001)


def f_ra_sender():
  global f_ra_move_speed, guider_correction_ra, ra_natural_running, ra_natural_speed

  while True:
    time.sleep(0.17)
    try:
      to_send = {
        'f_ra_move_speed': f_ra_move_speed,
        'guider_correction_ra': guider_correction_ra,
        'ra_natural_speed': ra_natural_speed,
        'f_dec_move_speed': f_dec_move_speed,
        'guider_correction_dec': guider_correction_dec,
      }
      to_send_json = json.dumps(to_send)
      f = open('/dev/shm/main_to_ra.json', 'w')
      f.write(to_send_json)
      f.close()
    except:
      pass

    if os.path.isfile('/dev/shm/ra_to_main.json'):
      f = open('/dev/shm/ra_to_main.json', 'r')
      try:
        params = json.loads(f.read())
        f.close()
        ra_natural_running = params['ra_natural_running']
      except:
        f.close()
        pass


#----------------------------------------------------

def f_dec_on():
  global p
  GPIO.output(p['dec_en'], 1)

def f_dec_off():
  global p
  GPIO.output(p['dec_en'], 0)

def f_dec_fastest():
  global p
  GPIO.output(p['dec_ms0'], 0)
  GPIO.output(p['dec_ms1'], 0)
  GPIO.output(p['dec_ms2'], 0)

def f_dec_fast():
  global p
  GPIO.output(p['dec_ms0'], 0)
  GPIO.output(p['dec_ms1'], 1)
  GPIO.output(p['dec_ms2'], 0)

def f_dec_slow():
  global p
  GPIO.output(p['dec_ms0'], 1)
  GPIO.output(p['dec_ms1'], 1)
  GPIO.output(p['dec_ms2'], 1)

def f_dec_left():
  global p
  GPIO.output(p['dec_dir'], 0)

def f_dec_right():
  global p
  GPIO.output(p['dec_dir'], 1)

def f_dec_step():
  global p
  GPIO.output(p['dec_step'], 1)
  time.sleep(4/1000000);
  GPIO.output(p['dec_step'], 0)
  time.sleep(4/1000000);

def f_dec_manual_move():
  global f_dec_move_speed
  global pi,p

  while True:
    if int(f_dec_move_speed) != 0:
      licznik = 0
      f_dec_on()
      last_f_dec_move_speed = 0
      pi.set_PWM_dutycycle(p['dec_step'], 50)
      while int(f_dec_move_speed) != 0:
        if f_dec_move_speed != last_f_dec_move_speed:
          if f_dec_move_speed > 0:
            f_dec_left()
          else:
            f_dec_right()
          if abs(f_dec_move_speed) > 98:
            f_dec_fastest()
            freq = abs(f_dec_move_speed)*6
          else:
            f_dec_fast()
            freq = abs(f_dec_move_speed)*12
          pi.set_PWM_frequency(p['dec_step'], freq)
          last_f_dec_move_speed = f_dec_move_speed

        else:
          time.sleep(0.1)
      f_dec_off()
      pi.set_PWM_dutycycle(p['dec_step'], 0)

    time.sleep(0.1)

#----------------------------------------------------

def f_ost_on():
  global p
  GPIO.output(p['ost_en'], 1)

def f_ost_off():
  global p
  GPIO.output(p['ost_en'], 0)

def f_ost_fastest():
  global p
  GPIO.output(p['ost_ms0'], 0)
  GPIO.output(p['ost_ms1'], 0)
  GPIO.output(p['ost_ms2'], 0)

def f_ost_fast():
  global p
  GPIO.output(p['ost_ms0'], 1)
  GPIO.output(p['ost_ms1'], 1)
  GPIO.output(p['ost_ms2'], 0)

def f_ost_slow():
  global p
  GPIO.output(p['ost_ms0'], 1)
  GPIO.output(p['ost_ms1'], 1)
  GPIO.output(p['ost_ms2'], 1)

def f_ost_left():
  global p
  GPIO.output(p['ost_dir'], 0)

def f_ost_right():
  global p
  GPIO.output(p['ost_dir'], 1)

def f_ost_step():
  global p
  GPIO.output(p['ost_step'], 1)
  time.sleep(4/1000000);
  GPIO.output(p['ost_step'], 0)
  time.sleep(4/1000000);

def f_ost_make_steps(side, steps):
  f_ost_fast()
  f_ost_on()

  if side == 'left':
    f_ost_left()
  else:
    f_ost_right()
  for i in range(steps*32):
    f_ost_step()
    time.sleep(0.002)
  f_ost_off()

def f_ost_manual_move():
  global f_ost_move_speed
  global pi,p

  while True:
    if int(f_ost_move_speed) != 0:
      licznik = 0
      f_ost_on()
      last_f_ost_move_speed = 0
      pi.set_PWM_dutycycle(p['ost_step'], 50)
      while int(f_ost_move_speed) != 0:
        if f_ost_move_speed != last_f_ost_move_speed:
          if f_ost_move_speed > 0:
            f_ost_left()
          else:
            f_ost_right()
          if abs(f_ost_move_speed) > 98:
            f_ost_fastest()
            freq = abs(f_ost_move_speed)*6
          else:
            f_ost_fast()
            freq = abs(f_ost_move_speed)*12
          pi.set_PWM_frequency(p['ost_step'], freq)
          last_f_ost_move_speed = f_ost_move_speed

        else:
          time.sleep(0.1)
      f_ost_off()
      pi.set_PWM_dutycycle(p['ost_step'], 0)

    time.sleep(0.1)


#############################################################################

class http_server(BaseHTTPRequestHandler):
  global f, cmd, t_ra, t_dec, ra_natural_speed
  global i1_real, i2_real, i3_real, i4_real, v1_real, v2_real, v3_real, v4_real
  global force_on_b1, force_on_b2, force_on_b3, force_on_b4
  global force_off_b1, force_off_b2, force_off_b3, force_off_b4
  global mah1, mah2, mah3, mah4
  global b1_state, b2_state, b3_state, b4_state
  global goto_working, after_meridian, guider_correction_ra, guider_correction_dec

  def do_GET(self):
    global f, cmd, ra_natural_speed
    global i1_real, i2_real, i3_real, i4_real, v1_real, v2_real, v3_real, v4_real
    global force_on_b1, force_on_b2, force_on_b3, force_on_b4
    global force_off_b1, force_off_b2, force_off_b3, force_off_b4
    global mah1, mah2, mah3, mah4
    global b1_state, b2_state, b3_state, b4_state
    global goto_working, last_ra_change, last_dec_change, last_radec_change_t, after_meridian, guider_correction_ra, guider_correction_dec
    try:
      if self.path == '/reboot':
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("ok", "utf-8"))
        out = subprocess.check_output(['/sbin/reboot'])
      elif self.path == '/restart':
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("ok", "utf-8"))
        out = subprocess.check_output(['/bin/systemctl', 'restart', 'teleskop'])
      elif self.path == '/shutdown':
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("ok", "utf-8"))
        out = subprocess.check_output(['/sbin/shutdown', 'now'])
      elif self.path == '/stats':
        f_read_battery()
        data = {
          'goto_working': goto_working,
          'ra_natual': ra_natural_speed,
          'position': get_pos_stat(),
          'last_ra_change': str(last_ra_change),
          'last_dec_change': str(last_dec_change),
          'last_radec_change_t': last_radec_change_t,
          'after_meridian': after_meridian,
          'guider_correction_ra': guider_correction_ra,
          'guider_correction_dec': guider_correction_dec,
          'battery': {
            '1': {
              'state':    b1_state,
              'voltage':  v1_real,
              'current':  i1_real,
              'force_on': force_on_b1,
              'force_off':force_off_b1,
              'used':     mah1,
            },
            '2': {
              'state':    b2_state,
              'voltage':  v2_real,
              'current':  i2_real,
              'force_on': force_on_b2,
              'force_off':force_off_b2,
              'used':     mah2,
            },
            '3': {
              'state':    b3_state,
              'voltage':  v3_real,
              'current':  i3_real,
              'force_on': force_on_b3,
              'force_off':force_off_b3,
              'used':     mah3,
            },
            '4': {
              'state':    b4_state,
              'voltage':  v4_real,
              'current':  i4_real,
              'force_on': force_on_b4,
              'force_off':force_off_b4,
              'used':     mah4,
            },
          },
        }
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes(json.dumps(data), "utf-8"))
      else:
        self.send_response(404)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("Not found", "utf-8"))
    except:
      self.send_response(500)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(bytes('FAIL', "utf-8"))

  def do_POST(self):
    global f, cmd

    try:
      content_len = int(self.headers.get('Content-Length'))
      post_body = json.loads(self.rfile.read(content_len))
      if post_body == {}:
        raise NameError('nok')
      cmd.put(post_body)

      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(bytes('ACK', "utf-8"))
    except:
      self.send_response(500)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(bytes('FAIL', "utf-8"))

def f_http_server():
  while True:
    try:
      server = HTTPServer(("0.0.0.0", 80), http_server)
      server.serve_forever()
    except:
      time.sleep(1)
      pass


#############################################################################

def f_ra_time_gen_events():
  global cmd_update_ra, after_meridian, ra_natural_speed

  curr_t = time.time()
  last_t = time.time()

  arcsec = 360.0 / 86400.0

  while True:
    curr_t = time.time()
    if ra_natural_speed == 'OFF':
      if after_meridian:
        mult = -1.0
      else:
        mult = 1.0
      diff = (float(curr_t) - float(last_t))
      diff_angle = Angle('00h00m01s') * diff
      cmd_update_ra.put([diff_angle, 'roll'])
      last_t = curr_t
    time.sleep(0.1)

def f_position_actual():
  global t_ra, t_dec, cmd_update_ra, cmd_update_dec

  while True:
    while not cmd_update_ra.empty():
      _cmd, method = cmd_update_ra.get()
      if method == 'set':
        t_ra = _cmd
      else:
        t_ra = t_ra + _cmd

    while not cmd_update_dec.empty():
      _cmd, method = cmd_update_dec.get()
      if method == 'set':
        t_dec = _cmd
      else:
        t_dec = t_dec + _cmd

    time.sleep(0.1)

def f_update_position(ra_corr = None, dec_corr = None, force = False):
  global cmd_update_ra, cmd_update_dec
  global last_ra_change, last_dec_change, last_radec_change_t
  global t_ra, t_dec

  if force:
    verb = 'set'
    last_ra_change = ra_corr - t_ra
    last_dec_change = dec_corr - t_dec
    last_radec_change_t = time.time()
  else:
    verb = 'roll'

  if ra_corr != None:
    cmd_update_ra.put([ra_corr, verb])

  if dec_corr != None:
    cmd_update_dec.put([dec_corr, verb])

#############################################################################

def get_pos_stat():
  global t_ra, t_dec, t_loc

  t = Time('J2021.04').now()
  actual = SkyCoord(ra=t_ra, dec=t_dec, frame='icrs')
  actual_altaz = actual.transform_to(AltAz(obstime=t,location=t_loc))
  out = {
    'alt': actual_altaz.alt.wrap_at(180 * u.deg).degree,
    'az':  actual_altaz.az.wrap_at(360 * u.deg).degree,
    'dec': t_dec.to_string(unit=u.deg),
    'ra': t_ra.to_string(unit=u.hour),
  }
  return out

#############################################################################

def pigpio_engine_steps(axis, steps, speed, update_pos, ra_dir = None, dec_dir = None):
  global pi, p, ra_natural_speed, ra_natural_running
  global t_ra, t_dec, after_meridian
  global dec_angles, ra_angles, goto_working
  goto_working = True

  last_ra_natural_speed = ra_natural_speed
  ra_natural_speed = 'OFF'
  while ra_natural_running:
    time.sleep(0.01)

  if not axis in ['ra', 'dec', 'both']:
    print("IGNORE! Wrong axis: " + str(axis))
    return

  if not speed in ['slow', 'fast', 'fastest']:
    print("IGNORE! Wrong speed: " + str(speed))
    return

  if axis == 'both':
    ra_multiply = 3
  else:
    ra_multiply = 1
  dec_multiply = 1

  ra_corr = 0
  dec_corr = 0

  if axis in ['ra', 'both']:
    f_ra_on()
    if int(ra_dir) == 1:
      f_ra_right()
    elif int(ra_dir) == -1:
      f_ra_left()
    else:
      print("ERR - wrong RA axis. RA set to OFF")
      ra_multiply = 0
    if speed == 'slow':
      f_ra_slow()
    elif speed == 'fast':
      f_ra_fast()
    else:
      f_ra_fastest()
    GPIO = p['ra_step']
    ra_corr = ra_angles[speed].deg*ra_multiply*steps*ra_dir
    ra_corr = Angle(Angle(ra_corr * u.deg).to_string(unit=u.hour))
  if axis in ['dec', 'both']:
    f_dec_on()
    if int(dec_dir) == -1:
      f_dec_right()
    elif int(dec_dir) == 1:
      f_dec_left()
    else:
      print("ERR - wrong DEC axis. DEC set to OFF")
      dec_multiply = 0
    if speed == 'slow':
      f_dec_slow()
    elif speed == 'fast':
      f_dec_fast()
    else:
      f_dec_fastest()
    GPIO = p['dec_step']
    dec_corr = dec_angles[speed].deg*dec_multiply*steps*dec_dir
    dec_corr = Angle(dec_corr * u.deg)

  if steps > 200:
    if axis in ['both', 'ra']:
      ramp = [[50,10], [150,20], [200,30], [600,40], [900,50], [1200,50]]
      ramp2 = ramp.copy()
      ramp2.reverse()
      max_speed = 1400
      steps_done = 2*(10+20+30+40+50+50)
    else:
      ramp = [[60,20], [200,30], [300,50]]
      ramp2 = ramp.copy()
      ramp2.reverse()
      max_speed = 450
      steps_done = 2*(50+30+20)
  else:
    if axis in ['both', 'ra']:
      ramp = [[600,steps]]
      ramp2 = []
      max_speed = 600
      steps_done = 0
    else:
      ramp = [[200,steps]]
      ramp2 = []
      max_speed = 200
      steps_done = 0

  steps_remaining = steps - steps_done
  while True:
    if steps_remaining > 65000:
      ramp.append([max_speed, 65000])
      steps_remaining = steps_remaining - 65000
    else:
      ramp.append([max_speed, steps_remaining])
      steps_remaining = 0

    if steps_remaining < 1:
      break
  for i in ramp2:
    print(i)
    ramp.append(i)

  pi.wave_clear()
  l = len(ramp)
  wid=[-1] * l

  for i in range(l):
    f = ramp[i][0]
    micros = int(500000/f)
    wf=[]
    if axis == 'both':
      wf.append(pigpio.pulse(1<<p['dec_step'], 0,       1))
      wf.append(pigpio.pulse(1<<p['ra_step'], 0,       micros))
      wf.append(pigpio.pulse(0,       1<<p['ra_step'], micros))
      wf.append(pigpio.pulse(1<<p['ra_step'], 0,       micros))
      wf.append(pigpio.pulse(0,       1<<p['ra_step'], micros))
      wf.append(pigpio.pulse(1<<p['ra_step'], 0,       micros))
      wf.append(pigpio.pulse(0,       1<<p['ra_step'], micros))
      wf.append(pigpio.pulse(0,       1<<p['dec_step'], 1))
    else:
      wf.append(pigpio.pulse(1<<GPIO, 0,       micros))
      wf.append(pigpio.pulse(0,       1<<GPIO, micros))
    pi.wave_add_generic(wf)
    wid[i] = pi.wave_create()


  chain = []
  for i in range(l):
    _steps = ramp[i][1]
    x = _steps & 255
    y = _steps >> 8
    chain += [255, 0, wid[i], 255, 1, x, y]

  pi.wave_chain(chain)

  while pi.wave_tx_busy():
    time.sleep(0.1)

  pi.wave_clear()
  if update_pos:
    if ra_corr == 0:
      ra_corr = None
    if dec_corr == 0:
      dec_corr = None
    f_update_position(ra_corr = ra_corr, dec_corr = dec_corr, force = False)
  goto_working = False
  ra_natural_speed = last_ra_natural_speed

#############################################################################

def change_telescope_pos(move = False, update_pos = False, axis = None, value = [], side = []):
  global t_ra, t_dec, t_loc, after_meridian, ra_natural_speed
  global dec_num_of_steps, cmd_dec_u, cmd_dec_d
  global ra_num_of_steps, cmd_ra_r, cmd_ra_l, dec_angles, ra_angles

  ra_moved = False

  last_ra_natural_speed = ra_natural_speed
  if move and (axis == 'altaz' or axis == 'radec' or ((axis == 'radec_rel') and value[0] != Angle(0*u.hour))):
    ra_natural_speed = 'OFF'
    while ra_natural_running:
      time.sleep(0.01)

  if len(value) != 2:
    print("IGNORE! Improper num of values in " + str(value))
    return
  if len(side) != 2 and axis == 'radec_rel':
    print("IGNORE! Improper num of sides in " + str(side))
    return

  for i in side:
    if not i in ['left', 'right', 'up', 'down', None]:
      print("IGNORE: Unknown side: " + str(i))
      return

  if axis == 'altaz' or axis == 'radec':

    if axis == 'altaz':
      t = Time('J2021.04').now()
      radec_place = SkyCoord(alt = Angle(value[0]), az = Angle(value[1]), obstime = t, frame = 'altaz', location = t_loc)
      _ra =  Angle(radec_place.icrs.ra.to_string(unit=u.hour))
      _dec = Angle(radec_place.icrs.dec.to_string(unit=u.deg))
    else:
      _ra =  Angle(value[0])
      _dec =  Angle(value[1])

    if move:
      ra_change = _ra - t_ra
      dec_change = _dec - t_dec


      if abs(ra_change) > Angle('12h00m00s'):
        if ra_change > 0:
          ra_change_corr = ra_change - Angle('24h0m0s')
        else:
          ra_change_corr = ra_change + Angle('24h0m0s')
        ra_change = ra_change_corr


      signum = [1,1]
      if ra_change < 0:
        signum[0] = -1
      if dec_change < 0:
        signum[1] = -1

      num_ra_steps = abs(int(float(ra_num_of_steps) / 256.0 / 360.0 * float(ra_change.deg)))
      num_dec_steps = abs(int(float(dec_num_of_steps) / 256.0 / 360.0 * float(dec_change.deg)))

      if min(int(num_ra_steps/3), num_dec_steps) > 1000:
        steps = min(int(num_ra_steps/3), num_dec_steps)
        outt = pigpio_engine_steps(axis='both', steps=steps, speed='fastest', update_pos=update_pos, ra_dir = signum[0], dec_dir = signum[1])
        print(outt)
        ra_moved = True
        num_ra_steps = num_ra_steps - 3*steps
        num_dec_steps = num_dec_steps - steps
      if num_ra_steps > 0:
        pigpio_engine_steps(axis='ra',  steps=num_ra_steps,  speed='fastest', update_pos=update_pos, ra_dir = signum[0], dec_dir = signum[1])
        ra_moved = True
      if num_dec_steps > 0:
        pigpio_engine_steps(axis='dec', steps=num_dec_steps, speed='fastest', update_pos=update_pos, ra_dir = signum[0], dec_dir = signum[1])
    else:
      f_update_position(ra_corr = _ra, dec_corr = _dec, force = True)

  elif axis == 'radec_rel':

    if after_meridian:
      signum = [-1,-1]
    else:
      signum = [1,1]

    if side[0] == 'right':
      signum[0] = signum[0] * -1
    if side[1] == 'left' or side[1] == 'down':
      signum[1] = signum[1] * -1

    if move:
      num_ra_steps = int(value[0] / ra_angles['fastest'])
      num_dec_steps = int(value[1] / dec_angles['fastest'])

      if min(num_ra_steps, num_dec_steps) > 1000:
        steps = min(num_ra_steps, num_dec_steps)
        pigpio_engine_steps(axis='both', steps=steps, speed='fastest', update_pos=update_pos, ra_dir = signum[0], dec_dir = signum[1])
        ra_moved = True
        num_ra_steps = num_ra_steps - steps
        num_dec_steps = num_dec_steps - steps
      if num_ra_steps > 0:
        pigpio_engine_steps(axis='ra',  steps=num_ra_steps,  speed='fastest', update_pos=update_pos, ra_dir = signum[0], dec_dir = signum[1])
        ra_moved = True
      if num_dec_steps > 0:
        pigpio_engine_steps(axis='dec', steps=num_dec_steps, speed='fastest', update_pos=update_pos, ra_dir = signum[0], dec_dir = signum[1])
    else:
      f_update_position(ra_corr = value[0] * signum[0], dec_corr = value[1] * signum[1], force = True)

  else:
    print("IGNORE! Unknown axis: " + str(axis))
    return

  if ra_moved:
    time.sleep(1)
    f_ra_slow()
    if after_meridian:
      f_ra_right()
    else:
      f_ra_left()
    _steps = 5500
    for i in range(_steps):
      f_ra_step()

  ra_natural_speed = last_ra_natural_speed

#############################################################################

def f_exec():
  global f_ra_move_speed, ra_natural_speed
  global f_dec_move_speed
  global f_ost_move_speed
  global force_on_b1, force_on_b2, force_on_b3, force_on_b4
  global force_off_b1, force_off_b2, force_off_b3, force_off_b4
  global mah1, mah2, mah3, mah4
  global after_meridian, guider_correction_ra, guider_correction_dec


  while True:
    try:
      msg = cmd.get()

      print(msg)


      mode = msg['mode']
      #continue

      #example = {
      #  'mode': 'ra_manual' | 'dec_manual' | 'ost_manual'
      #  'speed': int[-1000, 1000]
      #}
      if mode == 'ra_manual':
        _speed = int(msg['speed'])
        if after_meridian:
          _speed = int(msg['speed']) * -1
        else:
          _speed = int(msg['speed'])
        f_ra_move_speed = _speed

      elif mode == 'dec_manual':
        _speed = int(msg['speed'])
        if after_meridian:
          _speed = int(msg['speed']) * -1
        else:
          _speed = int(msg['speed'])
        f_dec_move_speed = _speed

      elif mode == 'ost_manual':
        f_ost_move_speed = int(msg['speed'])

      #example = {
      #  'mode': 'joystick'
      #  'side': 'right' | 'left' | 'up' | 'down'
      #  'angle': float
      #}
      elif mode == 'joystick':
        ang_ra = Angle(0*u.hour)
        ang_dec = Angle(0*u.deg)
        if msg['side'] in ['right', 'left']:
          ang_ra = Angle(msg['angle'] / 60 * u.hour)
          change_telescope_pos(move = True, update_pos = True, axis = 'radec_rel', value = [ang_ra, ang_dec], side = [msg['side'], None])
        else:
          ang_dec = Angle(msg['angle'] / 60 * u.deg)
          change_telescope_pos(move = True, update_pos = True, axis = 'radec_rel', value = [ang_ra, ang_dec], side = [None, msg['side']])

      #example = {
      #  'mode': 'ost_joystick'
      #  'side': 'right' | 'left'
      #  'steps': int
      #}
      elif mode == 'ost_joystick':
        f_ost_make_steps(side=msg['side'], steps=msg['steps'])

      #example = {
      #  'mode': 'altaz' | 'radec'
      #  'alt': float
      #  'az': float
      #  'ra': str
      #  'dec': str
      #  'move': 'true' | 'false'
      #  'update_pos': 'true' | 'false'
      #}
      elif mode == 'altaz':
        if 'alt' in msg.keys():
          alt = msg['alt']
        else:
          alt = None

        if 'az' in msg.keys():
          az = msg['az']
        else:
          az = None

        if 'move' in msg.keys():
          move = bool(msg['move'])
        else:
          move = False
        change_telescope_pos(move = msg['move'], update_pos = msg['update_pos'], axis = 'altaz', value = [alt, az])

      elif mode == 'radec':
        if 'ra' in msg.keys():
          ra = msg['ra']
        else:
          ra = None

        if 'dec' in msg.keys():
          dec = msg['dec']
        else:
          dec = None

        if 'move' in msg.keys():
          move = bool(msg['move'])
        else:
          move = False
        change_telescope_pos(move = msg['move'], update_pos = msg['update_pos'], axis = 'radec', value = [ra, dec])

        #example = {
        #  'mode': 'battery',
        #  'set': 'FORCED ON' | 'FORCED OFF' | 'AUTO',
        #  'battery': 1
        #}
      elif mode == 'battery':
        if msg['set'] == 'FORCED ON':
          if msg['battery'] == 1:
            force_on_b1 = True
            force_off_b1 = False
          elif msg['battery'] == 2:
            force_on_b2 = True
            force_off_b2 = False
          elif msg['battery'] == 3:
            force_on_b3 = True
            force_off_b3 = False
          elif msg['battery'] == 4:
            force_on_b4 = True
            force_off_b4 = False
        elif msg['set'] == 'FORCED OFF':
          if msg['battery'] == 1:
            force_on_b1 = False
            force_off_b1 = True
          elif msg['battery'] == 2:
            force_on_b2 = False
            force_off_b2 = True
          elif msg['battery'] == 3:
            force_on_b3 = False
            force_off_b3 = True
          elif msg['battery'] == 4:
            force_on_b4 = False
            force_off_b4 = True
        elif msg['set'] == 'AUTO':
          if msg['battery'] == 1:
            force_on_b1 = False
            force_off_b1 = False
          elif msg['battery'] == 2:
            force_on_b2 = False
            force_off_b2 = False
          elif msg['battery'] == 3:
            force_on_b3 = False
            force_off_b3 = False
          elif msg['battery'] == 4:
            force_on_b4 = False
            force_off_b4 = False
        elif msg['set'] == 'zero_mah':
          mah1= 0.0
          mah2= 0.0
          mah3= 0.0
          mah4= 0.0
      elif mode == 'ra_natural':
        ra_natural_speed = msg['speed']

        #example = {
        #  'mode': 'meridian',
        #  'value': 'after | before',
        #}
      elif mode == 'meridian':
        if msg['value'] == 'after':
          after_meridian = True
        elif msg['value'] == 'before':
          after_meridian = False

        #example = {
        #  'mode': 'guider_correction',
        #  'ra': <INT>,
        #  'dec': <INT>,
        #}
      elif mode == 'guider_correction':
        guider_correction_ra = int(msg['ra'])
        guider_correction_dec = int(msg['dec'])

    except:
      time.sleep(0.1)


#############################################################################

f_setup()

#try:
thread_list = []

t = threading.Thread(target=f_exec)
thread_list.append(t)

t = threading.Thread(target=f_ra_sender)
thread_list.append(t)

#t = threading.Thread(target=f_dec_manual_move)
#thread_list.append(t)

t = threading.Thread(target=f_ost_manual_move)
thread_list.append(t)

t = threading.Thread(target=f_http_server)
thread_list.append(t)

t = threading.Thread(target=f_battery)
thread_list.append(t)

t = threading.Thread(target=f_ra_time_gen_events)
thread_list.append(t)

t = threading.Thread(target=f_position_actual)
thread_list.append(t)


for thread in thread_list:
    thread.start()

for thread in thread_list:
    thread.join()
#except:
#  GPIO.cleanup()
#  sys.exit(1)

GPIO.cleanup()
