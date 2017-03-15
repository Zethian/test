import serial
import re

usart = serial.Serial(
    port = '/dev/ttyAMA0',
    baudrate = 19200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

WORDS=["JASPER"]

def isValid(text):
  return bool(re.search(r'\bjasper\b', text, re.IGNORECASE))
  
def handle(text, mic, profile):
  mic.say("HERE YOU GO")
  usart.write('You have recieved a new message')
