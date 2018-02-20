################################################################################
# Heart Rate Example
#
# Created: 2017-03-31 10:22:42.543966
#
################################################################################

import streams
from maxim.max30101 import max30101

streams.serial()

def check_for_beat(obj, smp):
    beat_detected = False
    obj._smp_buff[obj._cnt] = smp-obj._prev
    obj._prev=smp
    obj._cnt += 1
    if obj._cnt == 8:
        obj._cnt = 0
    smp_min = min(obj._smp_buff)
    if obj._smp_buff[(obj._cnt-4)&0x7] == smp_min and obj._smp_buff[(obj._cnt-5)&0x7] != 0 and smp_min <= obj._thr:
        if smp_min >= -2000 and smp_min <= -20:
            obj._thr = (obj._thr + obj._smp_buff[(obj._cnt-4)&0x7]*.6)/2
        beat_detected = True
        obj._smp_buff = [0]*8        
    return beat_detected

def detect_pulse(obj):   
    cnt = 0
    idx = 0
    hr_avg = 0
    HR_AVG_SIZE = 10
    rate = [0]*HR_AVG_SIZE
    while True:
        try:
            val = obj.read_raw_samples(6)
            hr = (val[3]<<16 | val[4]<<8 | val[5])
            cnt += 1
            res = check_for_beat(obj, hr)
            if res:
                dt = cnt*obj._dt
                cnt = 0
                bpm = 60000/dt
                if bpm < 255 and bpm > 20:
                    rate[idx] = bpm
                    idx += 1
                    if idx == HR_AVG_SIZE:
                        idx = 0
                    hr_avg = 0
                    for rr in rate:
                        hr_avg += rr
                    obj.hr_avg = hr_avg/HR_AVG_SIZE
            if cnt >= 3000/obj._dt:
                cnt = 0
                obj._thr = -20
                rate = [0]*HR_AVG_SIZE
                obj.hr_avg = 0
            obj.clear_fifo()
            sleep(obj._dt)
        except Exception as e:
            print("ErrorT", e)
            sleep(1000)

# create an instance of the MAX30101 class
try:
    # Setup sensor
    # This setup is referred to max30101 mounted on hexiwear device 
    # Power up and init sequence for max30101 inside hexiwear device
    pinPowerOn = D70
    digitalWrite(pinPowerOn,HIGH)
    pinMode(pinPowerOn,OUTPUT)
    # init sensor
    max = max30101.MAX30101(I2C0)
    print("start...")
    max.start()
    max._thr = -20
    max._dt = 50
    max._prev = 0
    max._smp_buff = [0]*8
    max._cnt = 0
    max.hr_avg = 0
    print("init...")
    max.init()
    # start a thread that reads and process raw data from the sensor
    thread(detect_pulse,max)
    print("Ready!")
    print("---------------------------")
except Exception as e:
    print(e)

while True:
    print("Heart Rate:", int(max.hr_avg))
    print("---------------------------")
    sleep(3000)