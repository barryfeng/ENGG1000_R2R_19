import time

class Data:
   def polar_to_cartesian(self, polar_array):
      print('memed')

   def gyroSetpoint(self, setpoint):
      return (setpoint + setpoint*0.02)

   def distSetpoint(self, setpoint):
      return (setpoint - setpoint*0.00)

   def positionFilter(self, positionSet):
      filteredSet = [[]]
      mean = sum(positionSet,0)/len(positionSet)
      uncertainty = abs((max(positionSet)-min(positionSet)/2))
      accepted_max = mean + uncertainty
      accepted_min = mean - uncertainty
      for value in positionSet:
         if value <= accepted_max and accepted_min <= value:
            filteredSet.append(value)
      return filteredSet

   def targetCenter(self, filteredSet):
      return (max(filteredSet) + min(filteredSet))/2

   def rolling_avg_filter(self, positionSet, currentPosition):
      filteredSet = [[]]
      mean = sum(positionSet,0)/len(positionSet)
      uncertainty = abs((max(positionSet)-min(positionSet)/2))
      accepted_max = mean + uncertainty
      accepted_min = mean - uncertainty
      for value in positionSet:
         if value <= accepted_max and accepted_min <= value:
            filteredSet.append(value)
      if filteredSet.count(currentPosition) == 1:
         return True
      else:
         return False 