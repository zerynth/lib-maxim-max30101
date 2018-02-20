"""

.. module:: max30101

*****************
MAX30101 Module
*****************

    This module contains the driver for MAXIM MAX30101 pulse oximetry and heart-rate monitor module.
    The MAX30101 is capable of direct I2C communication and can be set on 3 different operating mode (`datasheet <https://datasheets.maximintegrated.com/en/ds/MAX30101.pdf>`_).

"""
import i2c

# MAX30101 default slave address
MAX30101_I2CADDR       = 0x57

# MAX30101 registry addresses
# Status
MAX30101_INT_STATUS_1  = 0x00 
MAX30101_INT_STATUS_2  = 0x01 

MAX30101_INT_ENABLE_1  = 0x02
MAX30101_INT_ENABLE_2  = 0x03

# Fifo
MAX30101_FIFO_WR_PTR   = 0x04
MAX30101_OVF_COUNTER   = 0x05
MAX30101_FIFO_RD_PTR   = 0x06
MAX30101_FIFO_DATA     = 0x07

# Configuration
MAX30101_FIFO_CONF     = 0x08

MAX30101_MODE_CONF     = 0x09
MAX30101_SPO2_CONF     = 0x0A

MAX30101_LED_CONF_1    = 0x0C
MAX30101_LED_CONF_2    = 0x0D
MAX30101_LED_CONF_3    = 0x0E

MAX30101_PROX_MODE     = 0x10

MAX30101_MLM_CTRL_1    = 0x11
MAX30101_MLM_CTRL_2    = 0x12

# Temperature
MAX30101_TEMP_INT      = 0x1F
MAX30101_TEMP_FRA      = 0x20
MAX30101_TEMP_CONF     = 0x21

# Proximity function
MAX30101_PROX_THR      = 0x30

# Part ID
MAX30101_REV_ID        = 0xFE
MAX30101_PART_ID       = 0xFF

MAX30101_SR_50         = 0x0
MAX30101_SR_100        = 0x1
MAX30101_SR_200        = 0x2
MAX30101_SR_400        = 0x3
MAX30101_SR_800        = 0x4
MAX30101_SR_1000       = 0x5
MAX30101_SR_1600       = 0x6
MAX30101_SR_3200       = 0x7

MAX30101_SAMPLE_RATE = [
    MAX30101_SR_50,
    MAX30101_SR_100,
    MAX30101_SR_200,
    MAX30101_SR_400,
    MAX30101_SR_800,
    MAX30101_SR_1000,
    MAX30101_SR_1600,
    MAX30101_SR_3200
]

MAX3010_LED_PW_69      = 0x0
MAX3010_LED_PW_118     = 0x1
MAX3010_LED_PW_215     = 0x2
MAX3010_LED_PW_411     = 0x3

MAX3010_LED_PW = [
    MAX3010_LED_PW_69,
    MAX3010_LED_PW_118,
    MAX3010_LED_PW_215,
    MAX3010_LED_PW_411
]

MAX30101_LED_NONE      = 0x0
MAX30101_LED_RED       = 0x1
MAX30101_LED_IR        = 0x2
MAX30101_LED_GREEN     = 0x3

MAX30101_LED_CURRENT = [
    MAX30101_LED_NONE,
    MAX30101_LED_RED,
    MAX30101_LED_IR,
    MAX30101_LED_GREEN
]

MAX30101_MODE_HR       = 0x2
MAX30101_MODE_SPO2     = 0x3
MAX30101_MODE_MULTILED = 0x7

MAX30101_MODE = [
    MAX30101_MODE_HR,
    MAX30101_MODE_SPO2,
    MAX30101_MODE_MULTILED
]

MAX30101_INT_FULL      = 0x80
MAX30101_INT_DATA      = 0x40
MAX30101_INT_ALC       = 0x20
MAX30101_INT_PROX      = 0x10
MAX30101_INT_TEMP      = 0x02
MAX30101_INT_PWR       = 0x01


MAX30101_INT = {
    "full": MAX30101_INT_FULL,
    "data": MAX30101_INT_DATA,
    "alc":  MAX30101_INT_ALC,
    "prox": MAX30101_INT_PROX,
    "temp": MAX30101_INT_TEMP,  # write in MAX30101_INT_ENABLE_2_
    "pwr":  MAX30101_INT_PWR
}

MAX30101_AVG_1         = 0x0
MAX30101_AVG_2         = 0x1
MAX30101_AVG_4         = 0x2
MAX30101_AVG_8         = 0x3
MAX30101_AVG_16        = 0x4
MAX30101_AVG_32        = 0x5

MAX30101_AVG = [
    MAX30101_AVG_1,
    MAX30101_AVG_2,
    MAX30101_AVG_4,
    MAX30101_AVG_8,
    MAX30101_AVG_16,
    MAX30101_AVG_32
]

MAX30101_RANGE2048     = 0x0
MAX30101_RANGE4096     = 0x1
MAX30101_RANGE8192     = 0x2
MAX30101_RANGE16384    = 0x3

MAX30101_RANGE = [
    MAX30101_RANGE2048,
    MAX30101_RANGE4096,
    MAX30101_RANGE8192,
    MAX30101_RANGE16384
]


# Mode configurations
MODE_SHDN              = 0b10000000
MODE_RESET             = 0b01000000


class MAX30101(i2c.I2C):
    """

==============
MAX30101 Class
==============

.. class:: MAX30101(i2cdrv, addr=0x57, clk=400000)

    Creates an intance of the MAX30101 class.

    :param i2cdrv: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x75
    :param clk: Clock speed, default 400kHz. 

    Example: ::

        from maxim.max30101 import max30101

        ...

        m301 = max30101.MAX30101(I2C0)
        m301.start()
        m301.init()

        data = m301.read_raw_samples(6)

    """


    def __init__(self, drvname, addr=MAX30101_I2CADDR, clk=400000):

        i2c.I2C.__init__(self, drvname, addr, clk)
        
        # self._mean_hr_cnt = 0
        # self._mean_hr_lgt = 10
        # self._mean_hr_bff = [0]*self._mean_hr_lgt
        
        # self._smp_cnt = 0
        # self._smp_lgt = 10
        # self._smp_bff = [0]*self._smp_lgt
        # self._smp_thr = 0
        # self._smp_last = 0
        # self._dt = 50
        # self._monitor = False

        
    def init(self, mode = MAX30101_MODE_SPO2, adc_range = MAX30101_RANGE16384, sample_rate = MAX30101_SR_50, pulse_width = MAX3010_LED_PW_411, led_current = [0xFF,0xFF,0x00,0x00], proximity_thrs = 0, slot_multi = [0x00,0x00,0x00,0x00] ):
        """

.. method:: init(mode = 'spo2', adc_range = 3, sample_rate = 50, pulse_width = 411, led_current = [255,255,0,0], proximity_thrs = 0, slot_multi = [0,0,0,0] )
    
    Initialize the MAX30101. Default paramter values enable ``"spo2"`` - without proximity - mode with: maximum ADC range, sampling rate of 50 Hz, LED pulse width of 411us and maximum pulse amplitude for both red and IR LEDs. 

    :param mode: set the operating state of the MAX30101, default mode is ``spo2``.
    :param adc_range: sets the SpO2 sensor ADC's full-scale range, maximum range by default.
    :param sample_rate: sets the sampling rate, default is 50 Hz.
    :param pulse_width: LEDs pulse width in microsecond, default value is 411.
    :param led_current: sets red, IR, green and IR for proximity mode LEDs pulse amplitude, by default only red and IR red are set to maximum value.
    :param proximity_thrs: sets the threshold for the proximity mode.
    :param slot_multi: configuration parameter for ``"multi"`` operating mode.

.. note:: For details on available values for all the parameters see :func:`set_mode()`.
        
        """

        # self.clear_fifo()
        
        
        
        self.reset()
        sleep(50)
        self._set_led(led_current)
        
        self._set_multi_slots(slot_multi)
        
        self._set_led_pw(pulse_width)
        self._set_sample_rate(sample_rate)
        self._set_adc_rge(adc_range)
        self.set_fifo_rollover()
        self._set_prox_thr(proximity_thrs)
        self._set_mode(mode)
        
# Write data on register
    # def _write(self, addr, data):
    #     buffer = bytearray(1)
    #     buffer[0] = addr
    #     buffer.append(data)
    #     self.write(buffer)


# FIFO configuration
    def set_sample_averaging(self,n):
        """

FIFO configuration 
--------------------
    
    The MAX30101 stores the digital output data in a 32-deep circular FIFO within the IC. The sample size depends on the number of LED configured as active. As each led signal is stored as a 3-byte data, the FIFO width can be 3, 6, 9 or 12 bytes in size.


.. method:: set_sample_averaging(n)
    
    To reduce the amount of data throughput, ``n`` adjacent samples (in each individual channel) can be averaged and decimated on the chip by using this method.
    Accepted values for ``n`` are: 1,2,4,8,16 and 32.

        """  

        reg = self.write_read(MAX30101_FIFO_CONF,1)
        reg = reg[0] & 0x1F
        if n in MAX30101_AVG:
            val = n
        else:
            val = 0
        reg = reg | val<<5
        self.write_bytes(MAX30101_FIFO_CONF,reg)
        
    def set_fifo_rollover(self,ro = True):
        """

.. method:: set_fifo_rollover(ro = True)
    
    This method controls the behavior of the FIFO when the FIFO becomes completely filled with data. If ``ro`` is ``True``, the FIFO Address rolls over to zero and the FIFO continues to fill with new data. If ``ro`` is ``False``, then the FIFO is not updated until FIFO is read or the FIFO WRITE/READ pointer positions are changed.
    
        """  
        reg = self.write_read(MAX30101_FIFO_CONF,1)
        reg = reg[0] & 0xEF
        if ro:
            reg = reg | 0x10
        self.write_bytes(MAX30101_FIFO_CONF,reg)
    
    def set_fifo_afv(self,n):
        """

.. method:: set_fifo_afv(n)
    
    This method sets the trigger for the ``"full"`` interrupt. The interrupt triggers when there are ``n`` empty spaces left in FIFO.
    
        """  
        reg = self.write_read(MAX30101_FIFO_CONF,1)
        reg = reg[0] & 0xF0
        if n >= 0 and n <= 15:
            reg = reg | n
        self.write_bytes(MAX30101_FIFO_CONF,reg)
    
    
    def read_raw_samples(self,nbyte):
        """

.. method:: read_raw_samples(nbyte)
    
    Return a nbyte-long bytearray containing raw data read from the FIFO.
    
        """  
        v = self.write_read(MAX30101_FIFO_DATA,nbyte)
        return v
    

    def clear_fifo(self):
        """

.. method:: clear_fifo()
    
    This method set to zero the FIFO read and write pointers and the overflow counter.
    
        """  
        self.write_bytes(MAX30101_FIFO_RD_PTR,0x00)
        self.write_bytes(MAX30101_OVF_COUNTER,0x00)
        self.write_bytes(MAX30101_FIFO_WR_PTR,0x00)


# Interrupt status
    def enable_interrupt(self,sources=[]):
        """

Interrupt configuration 
-------------------------

.. method:: enable_interrupt(source)

    Set bit on enable interrupt registers corresponding to the selected source. 'sources' must be a list including one or more available sources. Available values for 'sources' are:

* ``"full"`` : in *spo2* or *hr* mode, this interrupt triggers when FIFO has a certain number of free spaces remaining.
* ``"data"`` : in *spo2* or *hr* mode, this interrupt triggers when there is a new sample in the data FIFO.
* ``"alc"`` : this interrupt triggers when the ambient light cancellation function of the SpO2/HR photodiode has reached its maximum limit, and therefore, ambient light is affecting the output of the ADC.
* ``"prox"`` : the proximity interrupt is triggered when the proximity threshold is reached, and SpO2/HR mode has begun.
* ``"temp"`` : when an internal die temperature conversion is finished, this interrupt is triggered so the processor can read the temperature data registers.

        """  
        try:
            for ss in sources:
                if ss in MAX30101_INT:
                    if MAX30101_INT[ss] == MAX30101_INT_PWR:
                        raise ValueError
                    if MAX30101_INT[ss] == MAX30101_INT_TEMP:
                        self.write_bytes(MAX30101_INT_ENABLE_2,0x02)
                    else:
                        reg = self.write_read(MAX30101_INT_ENABLE_1,1)
                        val = MAX30101_INT[ss]
                        reg = reg[0] | val
                        self.write_bytes(MAX30101_INT_ENABLE_1,reg)
            return self.read_triggered_interrupt()
        except Exception as e:
            print(e)
        
    def read_triggered_interrupt(self):
        """

.. method:: read_triggered_interrupt()

    This method needs to read which interrupt is triggered when more then one is enabled and
    returns a list containing triggered interrupts.

    .. note:: interrupt defines are shown in method above; this function can return "pwr" interrupt (triggered on every power-up) that is enabled by default and cannot be disabled.

        """
        interrupts = []
        data = self.write_read(MAX30101_INT_STATUS_1,2)
        for k,v in MAX30101_INT.items():
            if v == MAX30101_INT_TEMP:
                res = data[1] & v
            else:
                res = data[0] & v
            if res:
                interrupts.append(k)
        return interrupts



    def disable_interrupts(self):
        """

.. method:: disable_interrupts()
    
    Disable all interrupts.
        """ 
        self.write_bytes(MAX30101_INT_ENABLE_1,0x00)
        self.write_bytes(MAX30101_INT_ENABLE_2,0x00)



    
# Mode configuration
    def shutdown(self):
        """

Mode configuration 
------------------------

.. method:: shutdown()
    
    Put the part into a power-save mode. While in power-save mode, all registers retain their values, and write/read operations function as normal. All interrupts are cleared to zero in this mode.
        
        """
        reg = self.write_read(MAX30101_MODE_CONF,1)
        reg = reg[0] & 0x7F
        reg = reg | MODE_SHDN
        self.write_bytes(MAX30101_MODE_CONF, reg)
    
    def wake_up(self):
        """

.. method:: wake_up()
    
    Wake up the MAX30101 component.
        
        """
        reg = self.write_read(MAX30101_MODE_CONF,1)
        reg = reg[0] & 0x7F
        self.write_bytes(MAX30101_MODE_CONF, reg)
        
    def reset(self):
        """

.. method:: reset()
    
    All configuration, threshold, and data registers are reset to their power-on-state through a power-on reset.
        
        """
        self.write_bytes(MAX30101_MODE_CONF, MODE_RESET)
        cnt = 0
        while cnt < 5:
            res = self.write_read(MAX30101_MODE_CONF,1)[0]
            if (res & MODE_RESET) == 0:
                return True
            cnt += 1
            sleep(10)
        return False
    
    
    #'spo2','hr','multi'
    def set_mode(self, mode = 'spo2', adc_range = 3, sample_rate = 50, pulse_width = 411, led_current = [256,256,0,0], proximity_thrs = 0, slot_multi = [0,0,0,0] ):
        """

.. method:: set_mode(mode, adc_range, sample_rate, pulse_width, led_current, proximity_thrs, slot_multi)
    
    Set the operating mode of the MAX30101. Default paramter values are the same of :func:`init()`.
       
**Parameters:** 

* *mode* : set the operating state of the MAX30101. Available values are:

    * ``"hr"`` : Heart Rate mode - only red LED is used for conversion.
    * ``"spo2"`` : SpO2 mode - red and IR LEDs are used for conversion.
    * ``"multi"`` : Multi-LED mode - green, red and/or IR LEDs can be used for conversion.

* *adc_range* : sets the SpO2 sensor ADC's full-scale range as shown in the table below.

========= =============== =================
adc_range  LSB size (pA)   Full Scale (nA)
========= =============== =================
0           7.81             2048
1          15.63             4096
2          31.25             8192
3          62.5             16384
========= =============== =================

* *sample_rate* : sets the SpO2 effective sample rate. One sample consists of one IR pulse/conversion and one RED pulse/conversion. The sample rate and pulse width are related in that the sample rate sets an upper bound on the pulse width time. If the user selects a sample rate that is too high for the selected *pulse_width* setting, the highest possible sample rate is programmed instead. Available sampling rate values are: 50, 100, 200, 400, 800, 1000, 1600, 3200. 

* *pulse_width* : set the LED pulse width and indirectly sets the ADC resolution. All LEDs (IR, red and green) have the same pulse width.

=========== ================ =====================
pulse_width Pulse width (us) ADC Resolution (bits)
=========== ================ =====================
    69           68.95              15
    118          117.78             16
    215          215.44             17
    411          410.75             18
=========== ================ =====================

* *led_current* : sets the pulse amplitude for the LEDs. *led_current* = [red, ir, green, pilot]. The purpose of *pilot* is to set the LED power during the proximity mode, as well as in ``"multi"`` mode.

======================= ================
red, ir, green or pilot Led Current (mA)
======================= ================
    0                       0.0
    1                       0.2
    2                       0.4
    ...                     ...
    15                      3.1
    ...                     ...
    31                      6.4
    ...                     ...
    63                      12.5
    ...                     ...
    127                     25.4
    ...                     ...
    255                     50.0
======================= ================  

* *proximity_thrs* :  sets the IR ADC count that will trigger the beginning of ``"hr"`` or ``"spo2"`` mode and, if enabled, the prox interrupt. The threshold is defined as the 8 MSBs of the ADC count. For example, if proximity_thrs = 1, then an ADC value of 1023 (decimal) or higher triggers the prox interrupt. If proximity_thrs = 255, then only a saturated ADC triggers the interrupt.

* *slot_multi* :  In ``"multi"`` mode, each sample is split into up to four time slots, slot1 through slot4. *slot_multi* = [slot1, slot2, slot3, slot4]. These method control which LED is active with which amplitude in each time slot, making for a very flexible configuration. The slots should be enabled in order (i.e., slot1 should not be disabled if slot2 or slot3 are enabled).

======= ============== ===========================================
 slotX   Active Led     Led Pulse Amplitude (*led_current* value)
======= ============== ===========================================
    0    None           Off
    1    RED            red
    2    IR             ir
    3    GREEN          green
    4    None           Off
    5    RED            pilot
    6    IR             pilot
    7    GREEN          pilot
======= ============== ===========================================

        """

        self.clear_fifo()
        self._set_mode(mode)
        self._set_adc_rge(adc_range)
        self._set_sample_rate(sample_rate)
        self._set_led_pw(pulse_width)
        self._set_led(led_current)
        self._set_prox_thr(proximity_thrs)
        self._set_multi_slots(slot_multi)

        # self.init(mode, adc_range, sample_rate, pulse_width, led_current, proximity_thrs, slot_multi)




    def _set_mode(self,mode):
        reg = self.write_read(MAX30101_MODE_CONF,1)
        reg = reg[0] & 0xF8
            
        if mode in MAX30101_MODE:
            reg = reg | mode
            # self._clear_fifo()
        self.write_bytes(MAX30101_MODE_CONF,reg)
    
    
# SPO2 configuration
    def _set_adc_rge(self,n):
        reg = self.write_read(MAX30101_SPO2_CONF,1)
        reg = reg[0] & 0x9F
        if n >= 0 and n <= 3:
            reg = reg | n<<5
        self.write_bytes(MAX30101_SPO2_CONF,reg)

    def _set_sample_rate(self,v):
        reg = self.write_read(MAX30101_SPO2_CONF,1)
        reg = reg[0] & 0xE3
        if v in MAX30101_SAMPLE_RATE:
            reg = reg | v<<2
        self.write_bytes(MAX30101_SPO2_CONF,reg)
        
    def _set_led_pw(self,v):
        reg = self.write_read(MAX30101_SPO2_CONF,1)
        reg = reg[0] & 0xFC
        if v in MAX3010_LED_PW:
            reg = reg | v
        self.write_bytes(MAX30101_SPO2_CONF,reg)

# LED Pulse amplitude configuration
    def _set_led(self,led):
        self.write_bytes(MAX30101_LED_CONF_1,led[0])  # red
        self.write_bytes(MAX30101_LED_CONF_2,led[1])  # ir
        self.write_bytes(MAX30101_LED_CONF_3,led[2])  # green
        self.write_bytes(MAX30101_PROX_MODE,led[3])   # pilot

# Proximity mode threshold configuration
    def _set_prox_thr(self,v):
        self.write_bytes(MAX30101_PROX_THR,v)

# Multi led mode configuration
    def _set_multi_slots(self,slot):
        reg1 = slot[1]<<4 | slot[0]
        self.write_bytes(MAX30101_MLM_CTRL_1,reg1)
        reg2 = slot[3]<<4 | slot[2]
        self.write_bytes(MAX30101_MLM_CTRL_2,reg2)



# Temperature data    
    def enable_temperature(self):
        """

Temperature Data
------------------------

.. method:: enable_temperature()
    
    Initiates a single temperature reading from the temperature sensor.
        
        """  
        self.write_bytes(MAX30101_TEMP_CONF,0x01)

    def get_temperature(self):
        """

.. method:: get_temperature()
    
    Returns a float representing the last temperature reading in Celsius.
        
        """  
        t_reg_val = self.write_read(MAX30101_TEMP_INT, 2)
        t_raw = t_reg_val[0] << 4 | t_reg_val[1]
        t = float(t_raw) / 16
        if t_reg_val[0] > 127:
            t -= 256
        return t



# part and revision id    
    def get_part_id(self):
        """

Part ID
------------------------

.. method:: get_part_id()
    
    Returns the Part ID of the component.
        
        """
        data = self.write_read(MAX30101_PART_ID,1)
        return data[0]
        
    def get_revision_id(self):
        """

.. method:: get_revision_id()
    
    Returns the Revision ID of the component.
        
        """
        data = self.write_read(MAX30101_REV_ID,1)
        return data[0]
