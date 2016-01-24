#!/usr/bin/python

from bbio import *
from bbio.libraries.RotaryEncoder import RotaryEncoder
from multiprocessing import Value, Array
from bbio.libraries.SafeProcess import *
import random
from math import atan2, sin
from LCD import *


setPoint = Array('i', [0,0])

encoder_R = RotaryEncoder(RotaryEncoder.EQEP2b)
encoder_L = RotaryEncoder(RotaryEncoder.EQEP1)
LEFT_PWM = PWM1A
RIGHT_PWM = PWM2A
LEFT_ON = GPIO2_23
LEFT_LOW = GPIO2_22
RIGHT_ON = GPIO2_25
RIGHT_LOW = GPIO2_24
newDisplay = LCD(0x63)           

SPI0.begin()
SPI0.setMaxFrequency(0,50000)
SPI0.setMaxFrequency(1,50000)
SPI1.begin()
SPI1.setMaxFrequency(0,50000)
SPI1.setMaxFrequency(1,50000)


error_h = [0, 0]
i_e = 0
d_e = 0
threshold = 300

def adc_read(cs,ch):
    spidata0 = SPI0.transfer(cs,[1,(8+ch)<<4,0])
    data0 = ((spidata0[1] & 3) << 8) + spidata0[2]
    spidata1 = SPI1.transfer(cs,[1,(8+ch)<<4,0])
    data1 = ((spidata1[1] & 3) << 8) + spidata1[2]
    if cs == 0:
        return data0
    elif cs == 1:
        return data1

def sensor_reading(cs):
    reading = []
    for i in range(8):
        reading.append(adc_read(cs,i))
    return reading

def get_angle(f,r):
    front_max_idx = f.index(max(f))
    print 'front',front_max_idx, max(f)
    rear_max_idx = r.index(max(r))
    print 'rear',rear_max_idx,max(r)
    alpha = atan2(((front_max_idx - rear_max_idx) * 9.525 ) , 103)
    return alpha

def get_ratio(a):
    R = 121 / sin(abs(a))
    ratio = (R + 118) / (R - 118)
    return ratio

def readline(s):
    global threshold
    temp_data = 0
    temp_sum = 0
    for i in range(8):
        if s[i] >= threshold:
            temp_data += 1 * i * 10
            temp_sum += 1
    
    if temp_sum == 0:
        temp_sum = 1
        
    return temp_data / temp_sum

def find_error():
    global sensor_error, threshold
    sensor_error = 0
    front_sensor = sensor_reading(0)
    rear_sensor = sensor_reading(1)
    if max(front_sensor) > threshold:
        sensor_error = readline(front_sensor) - 35
        v_control(sensor_error, 0)
    
    if max(front_sensor) < threshold:
        sensor_error = 0
        set_speed(60, 60)
    ''' 
    if max(front_sensor) > threshold and max(rear_sensor) > threshold:
        #sensor_error = get_angle(front_sensor, rear_sensor)
        sensor_error = readline(front_sensor) - 35
        print sensor_error
        v_control(sensor_error, 0)
    elif max(front_sensor) > threshold and max(rear_sensor) < threshold:
        sensor_error = front_sensor.index(max(front_sensor))
        v_control(sensor_error, 1)
    elif max(front_sensor) < threshold and max(rear_sensor) > threshold:
        sensor_error = rear_sensor.index(max(rear_sensor))
        v_control(sensor_error, 2)
    elif max(front_sensor) < threshold and max(rear_sensor) < threshold:
        v_control(sensor_error, 3)
    '''
        
def v_control(e,t):
    global error_h, Kp, i_e, d_e
    basic_speed = 255 * 0.25
    error_h.append(e)
    error_h.pop(0)
    p_e = error_h[0]
    i_e += p_e
    d_e = error_h[1] - error_h[0]
    if t == 0:
        Kp = 1.2
        Ki = 0.01
        Kd = 2
        speed = v_pid(basic_speed, e, i_e, d_e, Kp, Ki, Kd)
        setPoint[0] = int(speed[0])
        setPoint[1] = int(speed[1])
        set_speed(speed[0], speed[1])
        #print speed
    elif t == 1:
        Kp = 5
        Kd = 0.1
        Ki = 0.1
        speed = v_pid(basic_speed, e, i_e, d_e, Kp, Ki, Kd)
        setPoint[0] = int(speed[0])
        setPoint[1] = int(speed[1])
    elif t == 2:
        Kp = 1
        Kd = 0.5
        Ki = 0.1
        speed = v_pid(basic_speed, e, i_e, d_e, Kp, Ki, Kd)
        setPoint[0] = int(speed[0])
        setPoint[1] = int(speed[1])
    elif t == 3:
        pass

def v_pid(b_s, e, p_e, d_e, Kp, Ki, Kd):
    delta_s = Kp * e + Kd * d_e + Ki * p_e
    s_l = b_s + delta_s
    if s_l >= 255:
        s_l = 255
    elif s_l <= 0:
        s_l = 0
    s_r = b_s - delta_s
    if s_r >= 255:
        s_r = 255
    elif s_r <= 0:
        s_r = 0
        
    return [s_l, s_r]

def motor_setup():
    pinMode(LEFT_ON, OUTPUT)
    pinMode(LEFT_LOW, OUTPUT)
    pinMode(RIGHT_ON, OUTPUT)
    pinMode(RIGHT_LOW, OUTPUT) 
    digitalWrite(RIGHT_ON, HIGH)
    digitalWrite(RIGHT_LOW, LOW)
    digitalWrite(LEFT_ON, HIGH)
    digitalWrite(RIGHT_LOW, LOW)

def motor_start():
    analogWrite(LEFT_PWM,1)    
    analogWrite(RIGHT_PWM,1)

def set_speed(l,r):
    pwmWrite(LEFT_PWM,l)
    pwmWrite(RIGHT_PWM,r)
    
def setup():
    encoder_R.setRelative()
    encoder_R.zero()
    encoder_R.setFrequency(100)
    encoder_L.setRelative()
    encoder_L.zero()
    encoder_L.setFrequency(100)
    motor_setup()
    motor_start()
    pid_p = SafeProcess(target = pid)
    #pid_p.start()
    
  
def pid():
    Kp = 2
    Ki = 0.088
    pre_error_L = 0
    integral_L = 0
    pre_error_R = 0
    integral_R = 0
    init_ouput_L = 255 * (setPoint[0] / 100)
    init_ouput_R = 255 * (setPoint[1] / 100)
    pre_output_L = init_ouput_L
    pre_output_R = init_ouput_R
  
    while(True):
        error_L = setPoint[0] - int(encoder_L.getPosition())
        error_R = setPoint[1] - int(encoder_R.getPosition())
        integral_L += error_L * 0.1
        integral_R += error_R * 0.1
        output_L = Kp * error_L + Ki * integral_L + pre_output_L
        output_R = Kp * error_R + Ki * integral_R + pre_output_R
        if output_L >= 255:
            output_L = 255
        elif output_L <= 0:
            output_L = 0
        if output_R >= 255:
            output_R = 255
        elif output_R <= 0:
            output_R = 0
        set_speed(output_L, output_R)
        pre_error_L = error_L
        pre_output_L = output_L
        pre_error_R = error_R
        pre_output_R = output_R
        print 'Setspeed_left  ' + str(setPoint[0]) + '  ' + encoder_L.getPosition()
        print 'Setspeed_right  ' + str(setPoint[1]) + '  ' + encoder_R.getPosition()
        delay(100)
      

def loop():
    ''' 
    global speed
    MAX_SPEED = 40
    a = get_angle()
    if a == 0:
        speed = [MAX_SPEED,MAX_SPEED]
    elif a > 0:
        speed = [MAX_SPEED / get_ratio(a),MAX_SPEED]
    elif a < 0:
        speed = [MAX_SPEED , MAX_SPEED / get_ratio(a)]
    
    setPoint[0] = int(speed[0])
    setPoint[1] = int(speed[1])
    log.Write('Angle: ' + str(a) + '  ' + 'Speed: ' + str(speed))
    delay(200)
    '''
    find_error()
    delay(10)

#log = open('Log.txt','w')



#user_input = newDisplay.keypad_input()
newDisplay.clear_screen()
newDisplay.write_text('Press 2/3/4/5 ')

while(1):
    user_input = newDisplay.keypad_input()
    if user_input == '2' or '3' or '4' or '5':
        newDisplay.clear_screen()
        newDisplay.write_text(user_input + ' Threshold' + str(int(user_input)*100))
        break
     
if user_input == '2':
    threshold = 200
elif user_input == '3':
    threshold = 300
elif user_input == '4':
    threshold = 400
elif user_input == '5':
    threshold = 500

newDisplay.set_cursor(2,1)
newDisplay.write_text('Press 1 to Go')

while(user_input != '1'):
    user_input = newDisplay.keypad_input()
    
newDisplay.clear_screen()
newDisplay.write_text('        GO!   ')

setup()
while(1):
    loop()
