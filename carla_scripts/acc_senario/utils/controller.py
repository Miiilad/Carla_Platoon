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
        self.kd = 10
        self.ki = 0.1
        self.integral = 0

        self.prev_input = 0

    def feedforward_ctrl(self, input=0):
        return 0
    
    def pid_ctrl(self, input=0):
        # derivative
        derivative = input - self.prev_input
        # integral
        self.integral += input
        self.integral = saturate(self.integral, -5, 5)
        value = self.kp * input + self.kd * derivative + self.ki * self.integral
        self.prev_input = input
        return value

    def control(self, input):
        pid_gain = self.pid_ctrl(input)
        feed_fowward_gain = self.feedforward_ctrl()
        value = pid_gain + feed_fowward_gain
        return saturate(value)