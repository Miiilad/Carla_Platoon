import numpy as np

configs = {
    "resolution": 1000,
    "attack_dense": 0.5,
    "seed_num": 200,
    "attack_type": "triangle",

    # for FDI attack
    "M": 100,
    "N": 10,
    "E1": 5,
    "E2": 10,
    "window_length": 10,

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
        
        self.attack_profile = None

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
            self.attack_profile = attack_profile

        return attack_profile

    def get_attacked_signal(self, signal, run_time, phase = 0, signal_size = 1):
        if self.attack_type == "random":
            n = (run_time % self.resolution)
            temp_profile = self.get_attack_profile(signal_size=signal_size) if self.attack_profile is None else self.attack_profile
            temp_profile = temp_profile[n+phase,:]
            signal = signal * temp_profile
        elif self.attack_type == "triangle":
            n = (run_time % self.resolution)
            temp_profile = self.get_attack_profile(signal_size=signal_size) if self.attack_profile is None else self.attack_profile
            temp_profile = temp_profile[n+phase,:]
            signal = signal * temp_profile
        return signal
    
def check_sum_of_squares(numbers, Umin, Umax, N, verbose=False):
    # N is the size of the sliding window
    for i in range(len(numbers) - N + 1):
        window_sum = np.sum(np.square(numbers[i:i+N]))
        if not (Umin <= window_sum <= Umax):
            if verbose:
                print(f"Sum of squares {window_sum} not within [{Umin}, {Umax}] at index {i}")
            if window_sum < Umin:
                return "Low"
            else:
                return "High"

    if verbose:      
        print("All windows satisfy the condition")
    return "done"

def generate_attacks(total_numbers:int, sliding_window:int, Upper_bound:int, Lower_bound:int, seed:int)->list:
    # Random seed
    n = seed
    # Scale factor for the Gaussian distribution
    scaler = 1

    np.random.seed(seed=n)
    valid_numbers = np.random.normal(0, scaler, sliding_window).tolist()

    while check_sum_of_squares(valid_numbers, Lower_bound, Upper_bound, sliding_window) != "done":
        if check_sum_of_squares(valid_numbers, Lower_bound, Upper_bound, sliding_window) == "Low":
            scaler = scaler_up(scaler)
        else:
            scaler = scaler_down(scaler)
        valid_numbers = np.random.normal(0, scaler, sliding_window).tolist()
        

    # Generate Gaussian distributed numbers
    while len(valid_numbers) < total_numbers:
        np.random.seed(n)
        n += 1
        # extend the list with one more number
        valid_numbers.append(np.random.normal(0, scaler))
        while check_sum_of_squares(valid_numbers, Lower_bound, Upper_bound, sliding_window) != "done":
            if check_sum_of_squares(valid_numbers, Lower_bound, Upper_bound, sliding_window) == "Low":
                scaler = scaler_up(scaler)
            else:
                scaler = scaler_down(scaler)
            valid_numbers[-1] = np.random.normal(0, scaler)

    return valid_numbers

def scaler_up(scaler):
    return scaler * 1.1

def scaler_down(scaler):
    return scaler * 0.9

class FDI_attacker():
    """
    Generate a sequence of length M of FDI attacks.
    Config parameters:
    - M: The total length of the sequence.
    - N: The window length within which the sum of absolute values of attacks must fall within [E1, E2].
    - E1, E2: The lower and upper bounds for the sum of attacks' energies within any N consecutive steps.
    - window_length: The length of the sliding window.
    Returns:
    - A list containing the sequence of FDI attacks.

    I want to generate a sequence of length M of FDI attacks. Within the sliding window of length N, the sum of square of values of attacks must fall within [E1, E2]. 
    I want to ensure that the sum of the absolute values of the last N attacks falls within [E1, E2]. 
    I want to generate random numbers with size = (resolution, signal_size). 
    I want to ensure the sum of the last N values falls within [E1, E2]. 
    I want to adjust the last attack values if the sum of the last N values falls outside [E1, E2].
    """
    def __init__(self, configs) -> None:
        for key, value in configs.items():
            setattr(self, key, value)

        self.attack_profile = None

    def get_attack_profile(self, signal_size = 1, seed_num = 200):
        N = self.window_length
        Umin = self.E1
        Umax = self.E2
        total_length = self.resolution
        n = self.seed_num

        self.attack_profile = np.zeros((total_length, signal_size))

        # generate profiles
        for i in range(signal_size):
            temp = generate_attacks(total_length, N, Umax, Umin, n)
            print(check_sum_of_squares(temp, Umin, Umax, N, verbose=True))
            self.attack_profile[:,i] = temp

    def get_attacked_signal(self, signal, run_time, phase = 0, signal_size = 1):
        n = (run_time % self.resolution)
        if self.attack_profile is None:
            self.get_attack_profile(signal_size=signal_size)
        temp_profile =  self.attack_profile[n+phase,:]
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
    signal_size = 3
    attacker = FDI_attacker(configs=configs)
    attacker.get_attack_profile(signal_size=signal_size)

    attack_profile = attacker.attack_profile
    
    # plot attack profile
    plt.figure()
    for i in range(signal_size):
        plt.subplot(signal_size, 1, i+1)
        plt.stairs(attack_profile[:,i], linewidth = 1.5)

        # grid legend
        plt.grid()
        plt.legend([f"signal {i}"])

        # x,y axis
        plt.xlim([0, attacker.M])
        plt.ylim([-1.2, 1.2])

    plt.show()

# main
if __name__ == "__main__":
    configs = configs
    configs["attack_dense"] = 0.8
    configs["resolution"] = 1000
    configs["attack_type"] = "triangle"

    # test_dos_attack(configs)
    test_fdi_attack(configs)


   