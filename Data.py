import time
import sys
from ev3dev2.led import Leds

led = Leds()

class Data:
   
   def polar_to_cartesian(self, polar_array):
      print('memed')

   def gyroSetpoint(self, setpoint):
      return (setpoint + setpoint*0.02)

   def distSetpoint(self, setpoint):
      return (setpoint - setpoint*0.00)

   def targetCenter(self, filteredSet):
      return (max(filteredSet) + min(filteredSet))/2

   def filter(self, positionSet, currentPosition):
      filteredSet = [[]]
      # mean = sum(positionSet[1],0)/len(positionSet[1])
      # uncertainty = abs((max(positionSet[1])-min(positionSet[1])/2))
      # accepted_max = mean + uncertainty
      # accepted_min = mean - uncertainty
      # tempSet = [position[1] for position in positionSet]
      # for value in tempSet:
      #    if value <= accepted_max and accepted_min <= value:
      #       filteredSet.append(value)
      
      if currentPosition in filteredSet:
         return True
      else:
         return False 

   def setLed(self, color):
      led.set_color('LEFT',color, pct=1)
      led.set_color('RIGHT',color, pct=1)

   def setLedOff(self):
      led.all_off()

   def average(self, val_1, val_2):
      return (val_1 + val_2)/2
   
   def cprint(self, statement):
      print(statement, file = sys.stderr)