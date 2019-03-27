import time

class Data:
   __instance = None
   @staticmethod 
   def getInstance():
      """ Static access method. """
      if Data.__instance == None:
         Data()
      return Data.__instance
   def __init__(self):
      if Data.__instance != None:
         raise Exception("SINGLETON CLASS")
      else:
         Data.__instance = self