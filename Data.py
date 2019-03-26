class Data:
    def comparator(self, setpoint, lower_bound, upper_bound, value):
        if setpoint - lower_bound <= value <= setpoint + upper_bound:
            return True
        else:
            return False
        