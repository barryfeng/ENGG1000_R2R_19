import time
import sys
from ev3dev2.led import Leds

led = Leds()

class Data:
   def gyroSetpoint(self, setpoint):
      return (setpoint + setpoint*0.02)

   def distSetpoint(self, setpoint):
      return (setpoint - setpoint*0.00)

   def targetCenter(self, filteredSet):
      return (max(filteredSet) + min(filteredSet))/2

   def setLed(self, color):
      led.set_color('LEFT',color, pct=1)
      led.set_color('RIGHT',color, pct=1)

   def setLedOff(self):
      led.all_off()

   def average(self, val_1, val_2):
      return (val_1 + val_2)/2
   
   def cprint(self, statement):
      print(statement, file = sys.stderr)