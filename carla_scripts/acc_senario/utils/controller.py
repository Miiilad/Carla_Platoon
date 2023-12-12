def pid_controller(input):
    kp = -20
    return kp * input

def saturate(value, min=0, max=1):
    if value > max:
        value = max
    elif value < min:
        value = min
    return value

class FeedForward_pid_Controller:
    def __init__(self, feedforward_gain=0):
        self.feedforward_gain = feedforward_gain
        self.kp = 5

    def feedforward_ctrl(self, input=0):
        return 0
    
    def pid_ctrl(self, input=0):
        value = self.kp * input
        return value

    def control(self, input):
        pid_gain = self.pid_ctrl(input)
        feed_fowward_gain = self.feedforward_ctrl()
        value = pid_gain + feed_fowward_gain
        return saturate(value)