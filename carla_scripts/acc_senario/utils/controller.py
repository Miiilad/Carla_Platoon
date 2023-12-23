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
    def __init__(self, P_value,I_value,D_value,feedforward_gain=0):
        # for feedforward
        self.feedforward_gain = feedforward_gain

        # for pid
        self.kp = P_value
        self.kd = D_value
        self.ki = I_value
        self.integral = 0

        self.prev_input = 0

        # for smooth fcn
        self.counter = 0
        self.smooth_len = 2
        self.list = []
        for i in range(self.smooth_len):
            self.list.append(0)

    def smooth_fcn(self, input):
        self.list[self.counter % self.smooth_len] = input
        self.counter += 1
        return sum(self.list) / self.smooth_len

    def feedforward_ctrl(self, input=0):
        return 0
    
    def pid_ctrl(self, input=0):
        # derivative
        derivative = input - self.prev_input
        # integral
        self.integral += input
        self.integral = saturate(self.integral, -5, 5)
        # pid
        value = self.kp * input + self.kd * derivative + self.ki * self.integral
        self.prev_input = input
        return value

    def control(self, input):
        # pid
        pid_gain = self.pid_ctrl(input)
        # feedforward
        feed_fowward_gain = self.feedforward_ctrl()
        # sum
        value = pid_gain + feed_fowward_gain
        # low pass filter
        #value = self.smooth_fcn(value)
        # saturate: the value of the throttle should be in [0, 1]
        return saturate(value)