import random


class attacker:
    def __init__(self, attack_dense = 0.0, attack_time = 5.0):
        self.attack_profile = [i < attack_dense*100 for i in range(100)]
        self.attack_dense = attack_dense
        self.attack_time = attack_time
        self.signal = 0

    def dos_attack(self, signal=None, time=0):
        x = int(time*100)%100
        attack_flag = self.attack_profile[x]
        if attack_flag and time > self.attack_time:
            # if the system is under attack, the signal keep the same as the previous signal
            return self.signal      
        else:
            # if the system is not under attack, the signal is input signal
            self.signal = signal # update the signal
            return signal