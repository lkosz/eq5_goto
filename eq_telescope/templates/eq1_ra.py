#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time, sys, os, threading, ujson as json, queue, numpy, pigpio, subprocess, board, busio

after_meridian = False

ra_natural_running = False
dec_natural_running = False
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
dec_natural_stop = False

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


def f_setup():
  global p

  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)

  for a in p.keys():
    GPIO.setup(p[a], GPIO.OUT)
    GPIO.output(p[a], 0)

  f_ra_on()
  f_dec_on()



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

def f_ra_natural():
  global step_delay_star, step_delay_sun, step_delay_moon, guider_correction_ra
  global ra_natural_speed, ra_natural_running

  next_step = float(time.time())
  #pom_last = time.time()
  while True:
    if time.time() >= next_step and ra_natural_speed != 'OFF':
      ra_natural_running = True
      f_ra_on()
      f_ra_slow()
      if after_meridian:
        f_ra_right()
      else:
        f_ra_left()

      if ra_natural_speed == "STAR":
        delay = step_delay_star
      elif ra_natural_speed == "SUN":
        delay = step_delay_sun
      elif ra_natural_speed == "MOON":
        delay = step_delay_moon

      f_ra_step()
      #pom_now = time.time()
      #if (pom_now-pom_last)*1000000 > 2000.0:
      #  print(pom_now-pom_last)
      #pom_last=pom_now

      next_step = float(next_step) + float(delay) + (float(guider_correction_ra)/1000000.0)
      _sleep = (next_step - time.time())/10.0
      if _sleep > 0.0:
        time.sleep(_sleep)
    else:
      if ra_natural_speed != 'OFF':
        time.sleep(step_delay_star/10.0)
      else:
        next_step = float(time.time())
        guider_correction_ra = 0
        guider_correction_dec = 0
        ra_natural_running = False
        time.sleep(0.3)

def f_ra_manual_move():
  global f_ra_move_speed, ra_natural_speed, ra_natural_running

  while True:
    if int(f_ra_move_speed) != 0:
      last_ra_natural_speed = ra_natural_speed
      ra_natural_speed = 'OFF'
      while ra_natural_running:
        time.sleep(0.01)
      licznik = 0
      f_ra_on()
      last_f_ra_move_speed = 0
      while int(f_ra_move_speed) != 0:
        ra_natural_speed = 'OFF'
        f_ra_on()
        if f_ra_move_speed > 0:
          f_ra_left()
        else:
          f_ra_right()
        if abs(f_ra_move_speed) > 98:
          f_ra_fastest()
          freq = abs(f_ra_move_speed)*12
        else:
          f_ra_fast()
          freq = abs(f_ra_move_speed)*12

        f_ra_step()
        time.sleep(1.0 / float(freq))
        last_f_ra_move_speed = f_ra_move_speed
      ra_natural_speed = last_ra_natural_speed
    else:
      time.sleep(0.1)

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


def f_dec_natural():
  global guider_correction_dec, dec_natural_running, dec_natural_stop

  next_step = float(time.time())
  pom_last = time.time()
  while True:
    _guider_correction_dec = guider_correction_dec
    if time.time() >= next_step and float(_guider_correction_dec) != 0.0 and dec_natural_stop == False:
      dec_natural_running = True
      f_dec_on()
      f_dec_slow()
      if after_meridian:
        if float(_guider_correction_dec) > 0.0:
          f_dec_right()
        else:
          f_dec_left()
      else:
        if float(_guider_correction_dec) > 0.0:
          f_dec_left()
        else:
          f_dec_right()

      f_dec_step()
      #pom_now = time.time()
      #print(pom_now-pom_last)
      #print()
      #pom_last=pom_now

      next_step = float(next_step) + abs(1.0/float(_guider_correction_dec))
      _sleep = (next_step - time.time())/10.0
      if _sleep > 0.0:
        time.sleep(_sleep)
    else:
      if _guider_correction_dec != 0.0:
        time.sleep(1/1000)
      else:
        next_step = float(time.time())
        guider_correction_dec = 0
        dec_natural_running = False
        time.sleep(0.3)

def f_dec_manual_move():
  global f_dec_move_speed, dec_natural_running, dec_natural_stop

  while True:
    if int(f_dec_move_speed) != 0:
      dec_natural_stop = True
      while dec_natural_running:
        time.sleep(0.01)
      licznik = 0
      f_dec_on()
      last_f_dec_move_speed = 0
      while int(f_dec_move_speed) != 0:
        f_dec_on()
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

        f_dec_step()
        time.sleep(1.0 / float(freq))
        last_f_dec_move_speed = f_dec_move_speed
    else:
      dec_natural_stop = False
      time.sleep(0.1)


#----------------------------------------------------

def f_parameters():
  global f_ra_move_speed, guider_correction_ra, ra_natural_speed
  global f_dec_move_speed, guider_correction_dec

  while True:
    time.sleep(0.11)
    if os.path.isfile('/dev/shm/main_to_ra.json'):
      f = open('/dev/shm/main_to_ra.json', 'r')
      try:
        params = json.loads(f.read())
        f.close()
        f_ra_move_speed = params['f_ra_move_speed']
        guider_correction_ra = params['guider_correction_ra']
        ra_natural_speed = params['ra_natural_speed']
        f_dec_move_speed = params['f_dec_move_speed']
        guider_correction_dec = params['guider_correction_dec']
      except:
        f.close()
        pass

    try:
      output_dict = {
        'ra_natural_running': ra_natural_running
      }
      output_dict_json = json.dumps(output_dict)
      f = open('/dev/shm/ra_to_main.json', 'w')
      f.write(output_dict_json)
      f.close()
    except:
      print('aa')
      pass



############################################
f_setup()

thread_list = []

t = threading.Thread(target=f_parameters)
thread_list.append(t)

t = threading.Thread(target=f_ra_natural)
thread_list.append(t)

t = threading.Thread(target=f_ra_manual_move)
thread_list.append(t)

t = threading.Thread(target=f_dec_manual_move)
thread_list.append(t)

t = threading.Thread(target=f_dec_natural)
thread_list.append(t)

for thread in thread_list:
    thread.start()

for thread in thread_list:
    thread.join()
