import numpy as np

configs = {
    "resolution": 1000,
    "attack_dense": 0.5,
    "seed_num": 200,
    "attack_type": "triangle",
    "power": 1.0,
    "split_rate": 0.7,

    "plot": True,
}

class DosAttacker:
    def __init__(self, configs):
        '''
        configs: dict of configs
        signal_size: int, number of signals
        resolution: int, resolution of attack profile
        attack_dense: float, dense of attack
        attack_on: bool, if attack is on
        seed_num: int, seed number
        attack_type: str, type of attack, including "random", "triangle"
        '''
        for key, value in configs.items():
            setattr(self, key, value)

    def get_attack_profile(self, signal_size = 1, seed_num = 200):
        if self.attack_type == "random":
            # generate random seed
            seed = seed_num
            random_numbers = np.random.RandomState(seed).rand(self.resolution, signal_size)
            temp = random_numbers < self.attack_dense
            # convert to numpy array
            attack_profile = np.array(temp, dtype = np.float64)
        elif self.attack_type == "triangle":
            # generate triangle wave from 0 to 1
            attack_profile = np.zeros((self.resolution, signal_size))
            for i in range(int(self.resolution/2)):
                attack_profile[i,:] = i / (self.resolution/2)
            for i in range(int(self.resolution/2), self.resolution):
                attack_profile[i,:] = 1 - (i - int(self.resolution/2)) / (self.resolution/2)
            
            # generate 0 and 1
            temp = attack_profile > self.attack_dense
            # convert to numpy array
            attack_profile = np.array(temp, dtype = np.float64)

        return attack_profile

    def get_attacked_signal(self, signal, run_time, phase = 0, signal_size = 1):
        if self.attack_type == "random":
            n = (run_time % self.resolution)
            temp_profile = self.get_attack_profile(signal_size=signal_size)
            temp_profile = temp_profile[n+phase,:]
            signal = signal * temp_profile
        elif self.attack_type == "triangle":
            n = (run_time % self.resolution)
            temp_profile = self.get_attack_profile(signal_size=signal_size)
            temp_profile = temp_profile[n+phase,:]
            signal = signal * temp_profile
        return signal

class FDI_attacker():
    '''
    FDI attacker:
    1. Given a value of "power" of the attacker, the attack power can be reflected on this value.
    2. numpy generate random seed to make sure the attack is random and we get the same result every time.
    '''
    def __init__(self, configs) -> None:
        for key, value in configs.items():
            setattr(self, key, value)
        
    def get_attack_profile(self, signal_size = 1, seed_num = 200):
        # gererate random numbers with size = (resolution, signal_size)
        weights_profile = np.random.RandomState(seed_num).rand(self.resolution, signal_size)*(1-self.split_rate) + self.split_rate
        sign_profile = np.random.RandomState(seed_num+1).rand(self.resolution, signal_size)-0.5
        sign_profile = np.sign(sign_profile)
        
        self.attack_profile = weights_profile * sign_profile*self.power

        if self.plot:
            plt.figure()
            for i in range(signal_size):
                x = np.linspace(0, self.resolution, self.resolution)
                plt.subplot(signal_size, 1, i+1)
                plt.scatter(x, self.attack_profile[:,i])

                # grid legend
                plt.grid()
                plt.legend([f"signal {i}"])

                # x,y axis
                plt.xlim([0, self.resolution])
                plt.ylim([-1.2, 1.2])
            plt.show()

        return self.attack_profile
    
    def get_attacked_signal(self, signal, run_time, phase = 0, signal_size = 1):
        n = (run_time % self.resolution)
        temp_profile = self.get_attack_profile(signal_size=signal_size)
        temp_profile = temp_profile[n+phase,:]
        signal = signal + temp_profile
        return signal

    

# >>>>>>>>>>>>>>>>>>>>>>>>>>> test >>>>>>>>>>>>>>>>>>>>>>>>>>>
TEST_ATTACK_PROFILE = True
TEST_SIGNAL = True

import matplotlib.pyplot as plt


def test_dos_attack(configs):
    attacker = DosAttacker(configs=configs)

    if TEST_ATTACK_PROFILE:
        plt.figure()
        # subplot
        signal_size = 3
        for i in range(signal_size):
            plt.subplot(signal_size, 1, i+1)
            attack_profile = attacker.get_attack_profile(signal_size=signal_size)
            plt.stairs(attack_profile[:,i], linewidth = 1.5)

            # grid legend
            plt.grid()
            plt.legend([f"signal {i}"])

            # x,y axis
            plt.xlim([0, attacker.resolution])
            plt.ylim([-0.2, 1.2])

    if TEST_SIGNAL:
        test_signal_length = attacker.resolution*10
        # generate a signal with sine wave
        signal = np.zeros((test_signal_length, signal_size))
        for i in range(signal_size):
            signal[:,i] = np.sin(np.linspace(0, 10*np.pi, test_signal_length))
        
        # implement attack
        for i in range(test_signal_length):
            temp_signal = signal[i,:]
            signal[i,:] = attacker.get_attacked_signal(temp_signal, i)

        plt.figure()
        # subplot
        for i in range(signal_size):
            plt.subplot(signal_size, 1, i+1)
            plt.stairs(signal[:,i])

            # grid legend
            plt.grid()
            plt.legend([f"signal {i}"])
    
    plt.show()

def test_fdi_attack(configs):
    attacker = FDI_attacker(configs=configs)
    attacker.get_attack_profile(3)


# main
if __name__ == "__main__":
    configs = configs
    configs["attack_dense"] = 0.8
    configs["resolution"] = 1000
    configs["attack_type"] = "triangle"

    # test_dos_attack(configs)
    test_fdi_attack(configs)


   