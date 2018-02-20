import socket, network, utime
from machine import Pin, I2C, ADC, Timer
import ustruct as struct
from neopixel import NeoPixel
import time, machine
import sys

#######################################
# IQ Light(c) & ESP32 & Network Watch
# Kosuke Miyaki @ 2018
# License :: Apache 2.0
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.

########## Color wheel ##########
#(r, g, b)
color_array = (
    (146,   7, 131),
    ( 96,  25, 134),
    ( 29,  32, 136),
    (  0, 104, 183),
    (  0,  71, 157),
    (  0, 134, 183),
    (  0, 160, 233),
    (  0, 160, 193),
    (  0, 158, 150),
    (  0, 155, 107),
    (  0, 153,  68),
    ( 34, 172,  56),
    (143, 195,  31),
    (207, 219,   0),
    (255, 251,   0),
    (243, 152,   0),
    (252, 200,   0),
    (235,  97,   0),
    (230,   0,  18),
    (230,   0,  51),
    (229,   0,  79),
    (229,   0, 106),
    (228,   0, 127),
    (190,   0, 129),
)

########## html ##########
html = """<!DOCTYPE html>
<html>
    <head>
        <meta charset='utf-8'>
        <title>AP Set from ESP-Light</title>
    </head>
    <body>
    <center>
        <h1> Setting Form </h1>
            <h2> from Real Time ESP-Light </h2> <br>
        <form>
            <p> Access Point Name <br>
            <input type="text", name="point_name", size=30 ></p>
            <p> Password <br>
            <input type="password", name="point_pwd", size=30></p>
            <p><br><input type="submit", value="submit"></p>
        </form>
    </center>
    </body>
</html>
"""

########## function ##########
def wifi_connect():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print ('Connecting to Network......')
        sta_if.active(True)
        try :
            f = open('text.txt', 'r')
            read_val = f.readlines()
            ap_name = read_val[0].rstrip()
            pw_name = read_val[1].rstrip()
            print ('ap = %s, pw = %s' % (ap_name, pw_name) )
            sta_if.connect(ap_name, pw_name)
        except :
            return False
        print ('Waiting Connection......')
        utime.sleep(5)
        if not sta_if.isconnected():
            sta_if.active(False)
            return False
        print ('Network config:',sta_if.ifconfig() )
    return True


def time_config():
    NTP_DELTA = 2208988800 + 946684800 - 32400
    #offset NTP time
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1b
    print ('Try to configure NTP')
    try :
        ntp_addr = socket.getaddrinfo("ntp.nict.jp", 123)[0][-1]
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        res = s.sendto(NTP_QUERY, ntp_addr)
        msg = s.recv(48)
        s.close()
    except :
        raise Exception
        return 0, 0
    val = struct.unpack("!I", msg[40:44])[0]
    u_time = utime.time()
    n_time = val - NTP_DELTA
    return u_time, n_time

def time_get(u_time, n_time):
    return utime.localtime(utime.time() - u_time + n_time)

def lcd_init():
    i2c.writeto(addr, setbuff)
    time.sleep_ms(1)

def lcd_clear():
    buff = bytearray(2)
    buff[0] = 0x00
    buff[1] = 0x01
    i2c.writeto(addr, buff)
    time.sleep_ms(1)

def lcd_linechange():
    buff = bytearray(2)
    buff[0] = 0x00
    buff[1] = 0xC0
    i2c.writeto(addr, buff)
    time.sleep_ms(1)

def lcd_write(mystr) :
    buff = bytearray(mystr)
    buff[0] = 0x40
    i2c.writeto(addr, buff)
    time.sleep_ms(1)

def callback_sw1(self):
    global is_apmode, mode, disable_sw1
    is_apmode = 1
    mode = 1
    print ('push sw1')

def callback_sw2(self):
    global mode, disable_sw2
    if disable_sw2 == 0 :
        if mode == 2 or is_apmode == 1:
            mode = 1
        else :
            mode = 2
        print ('push sw2 and now is mode %s' % mode)
        disable_sw2 = 1

def callback_disp(self):
    if is_apmode == 0 :
        #display date
        now = time_get(update_time, ntp_time)
        lcd_clear()
        lineup = ':'.join(map(str,now[0:3]))
        lcd_write(' %s' % lineup[2:] )
        lcd_linechange()
        lcd_write(' %s' % ':'.join(map(str, now[3:6])) )
    global disable_sw2
    disable_sw2 = 0


########## define pin ##########
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)
pin34 = Pin(34, Pin.IN)
sw1 = Pin(18, Pin.IN, Pin.PULL_UP)
sw2 = Pin(19, Pin.IN, Pin.PULL_UP)
adc = ADC(pin34)
adc.atten(ADC.ATTN_11DB)
tim = Timer(-1)
np = NeoPixel(Pin(27), 10)

########## global variable #########
# real time from NICT, unit::sec
ntp_time = 0
update_time = 0
flag_time = 0
# I2C address of the lcd
addr = 62
# Internal State variant
is_apmode = 0
mode = 2
c_error = False
is_boot = 0
disable_sw2 = 0
# Constant
loop_ms = 30
# I2C set command buff
setbuff = bytearray(9)
setbuff[0] = 0x38
setbuff[1] = 0x39
setbuff[2] = 0x14
setbuff[3] = 0x70
setbuff[4] = 0x56
setbuff[5] = 0x6C
setbuff[6] = 0x38
setbuff[7] = 0x0C
setbuff[8] = 0x01

########## main ##########
while True:
    #endless loop
    #void setup
    print ('(re)booting...')
    lcd_init()
    isconnect = wifi_connect()
    if isconnect == False:
        c_error = True
        mode = 1
        is_apmode = 1
        print ('Wifi_Boot Error')
    else :
        try :
            update_time, ntp_time = time_config()
            mode = 2
            c_error = False
            is_apmode = 0
            print ('No Problem in Boot')
        except :
            c_error = True
            mode = 1
            is_apmode = 1
            print ('NTP Get Error')
    if is_boot == 0 :
        sw1.irq(trigger=Pin.IRQ_FALLING, handler=callback_sw1)
        sw2.irq(trigger=Pin.IRQ_FALLING, handler=callback_sw2)
        tim.init(period=1000, mode=Timer.PERIODIC, callback=callback_disp)
        is_boot = 1
        print ('is_boot is set')

    #void loop
    while True :
        if is_apmode == 1 :
            for i in range(0, 10) :
                np[i] = (0, 0, 0)
            np.write()
            ###AP Mode Activate###
            print ('ap_mode Activating.......')
            lcd_clear()
            lcd_write('  AP MODE')
            ap_if = network.WLAN(network.AP_IF)
            if not ap_if.active():
                print ('Activating AP mode.......')
                try :
                    ap_if.active(True)
                except :
                    break
                print (ap_if.ifconfig())
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind(('',80))
            s.listen(1)
            while True:
                cl, addr = s.accept()
                print('client connected from', addr)
                request = cl.recv(1024)
                request = str(request)
                print('received data is', request)

                ap_flag = request.find('?point_name')
                pw_flag = request.find('&point_pwd')
                end_flag = request.find(' HTTP')
                if ap_flag != -1:
                    if pw_flag != -1:
                        if end_flag != -1:
                            ap_name = request[ap_flag+12:pw_flag]
                            pw_name = request[pw_flag+11:end_flag]
                            break
                cl.send(html)
                cl.close()

            print ('AP&PW Name', ap_name, pw_name)
            write_val = ap_name + '\r\n' + pw_name + '\r\n'
            f = open('text.txt', 'w')
            f.write(write_val)
            f.close()
            print ('Write to text')
            machine.reset()
            ### AP MODE ACTIVATE END ###

        ### NTP Configure ###
        if flag_time > ( 600000 / loop_ms ) :
            try :
                update_time, ntp_time = time_config()
                print( 'time configure @ %s' % ntp_time)
                flag_time = 0
            except :
                machine.reset()
        else :
            flag_time = flag_time + 1

        ### NeoPixel Control ###
        vol = adc.read() / 4095.0
        vol_255 = int( vol * 255.0 )
        if mode == 1 :
            #white
            for i in range(0, 10) :
                np[i] = (vol_255, vol_255, vol_255)
        else :
            #with hour
            now = time_get(update_time, ntp_time)
            color = [0, 0, 0]
            for i in range(0, 3) :
                color[i] = int( color_array[now[3]][i] * vol )
            for i in range(0, 10) :
                if i % 2 == 0 :
                    np[i] = tuple(color)
                else :
                    np[i] = (vol_255, vol_255, vol_255)
        np.write()
        time.sleep_ms(loop_ms)
