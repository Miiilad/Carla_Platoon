import numpy as np

configs = {
    "signal_size": 3,
    "signal_length": 1000,
    "attack_dense": 0.5,
    "attack_on": True,
    "seed_num": 200,
    "attack_type": "triangle",
}

class DosAttacker:
    def __init__(self, configs):
        '''
        configs: dict of configs
        signal_size: int, number of signals
        signal_length: int, length of signal
        attack_dense: float, dense of attack
        attack_on: bool, if attack is on
        seed_num: int, seed number
        attack_type: str, type of attack, including "random", "triangle"
        '''
        for key, value in configs.items():
            setattr(self, key, value)
        self.count = 0
        self.get_attack_profile(seed_num = self.seed_num, attack_type = self.attack_type)

    def get_attack_profile(self, attack_type = "random", seed_num = 200):
        if attack_type == "random":
            # generate random seed
            seed = seed_num
            random_numbers = np.random.RandomState(seed).rand(self.signal_length, self.signal_size)
            print(random_numbers.size)
            temp = random_numbers < self.attack_dense
            # convert to numpy array
            self.attack_profile = np.array(temp, dtype = np.float64)
        elif attack_type == "triangle":
            # generate triangle wave from 0 to 1
            self.attack_profile = np.zeros((self.signal_length, self.signal_size))
            for i in range(int(self.signal_length/2)):
                self.attack_profile[i,:] = i / (self.signal_length/2)
            for i in range(int(self.signal_length/2), self.signal_length):
                self.attack_profile[i,:] = 1 - (i - int(self.signal_length/2)) / (self.signal_length/2)
            
            # generate 0 and 1
            temp = self.attack_profile > self.attack_dense
            # convert to numpy array
            self.attack_profile = np.array(temp, dtype = np.float64)

    def attack(self, signal):
        if self.attack_on:     
            signal = self.get_attacked_signal(signal)
        return signal
    
    def get_attacked_signal(self, signal):
        self.count += 1
        if self.attack_type == "random":
            n = (self.count % self.signal_length)
            temp_profile = self.attack_profile[n]
            signal = signal * temp_profile
        elif self.attack_type == "triangle":
            n = (self.count % self.signal_length)
            temp_profile = self.attack_profile[n]
            signal = signal * temp_profile
        return signal


# >>>>>>>>>>>>>>>>>>>>>>>>>>> test >>>>>>>>>>>>>>>>>>>>>>>>>>>
TEST_ATTACK_PROFILE = False
TEST_SIGNAL = True


# main
if __name__ == "__main__":
    configs = configs
    configs["attack_dense"] = 0.4
    configs["signal_length"] = 100

    attacker = DosAttacker(configs=configs)
    import matplotlib.pyplot as plt
    if TEST_ATTACK_PROFILE:
        plt.figure()
        # subplot
        for i in range(attacker.signal_size):
            plt.subplot(attacker.signal_size, 1, i+1)
            plt.stairs(attacker.attack_profile[:,i], linewidth = 1.5)

            # grid legend
            plt.grid()
            plt.legend([f"signal {i}"])

            # x,y axis
            plt.xlim([0, attacker.signal_length])
            plt.ylim([-0.2, 1.2])

    if TEST_SIGNAL:
        test_signal_length = attacker.signal_length*10
        # generate a signal with sine wave
        signal = np.zeros((test_signal_length, attacker.signal_size))
        for i in range(attacker.signal_size):
            signal[:,i] = np.sin(np.linspace(0, 10*np.pi, test_signal_length))
        
        # implement attack
        for i in range(test_signal_length):
            temp_signal = signal[i,:]
            signal[i,:] = attacker.attack(temp_signal)

        plt.figure()
        # subplot
        for i in range(attacker.signal_size):
            plt.subplot(attacker.signal_size, 1, i+1)
            plt.stairs(signal[:,i])

            # grid legend
            plt.grid()
            plt.legend([f"signal {i}"])
    
    plt.show()