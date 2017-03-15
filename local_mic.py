# -*- coding: utf-8-*-
"""
A drop-in replacement for the Mic class that allows for all I/O to occur
over the terminal. Useful for debugging. Unlike with the typical Mic
implementation, Jasper is always active listening with local_mic.
FROM JASPERPROJECT
"""
import serial

usart = serial.Serial(
      port = '/dev/ttyAMA0',
      baudrate = 19200,
      parity = serial.PARITY_NONE,
      stopbits = serial.STOPBITS_ONE,
      bytesize = serial.EIGHTBITS,
      timeout = 1
)

class Mic:
    prev = None

    def __init__(self, speaker, passive_stt_engine, active_stt_engine):
        return

    def passiveListen(self, PERSONA):
        return True, "JASPER"

    def activeListenToAllOptions(self, THRESHOLD=None, LISTEN=True,
                                 MUSIC=False):
        return [self.activeListen(THRESHOLD=THRESHOLD, LISTEN=LISTEN,
                                  MUSIC=MUSIC)]

    def activeListen(self, THRESHOLD=None, LISTEN=True, MUSIC=False):
        if not LISTEN:
            return self.prev
"""
HÄR HAR JAG OCKSÅ ÄNDRAT 
"""   
        uart = usart.readline()
        while uart == '':
          uart = usart.readline()
          
        if uart != '':
          print uart
          #input = raw_input("YOU: ")
          input = uart
          self.prev = input
          return input
        else:
          print 'Something went wrong'
          return self.prev

    def say(self, phrase, OPTIONS=None):
        print("JASPER: %s" % phrase)
