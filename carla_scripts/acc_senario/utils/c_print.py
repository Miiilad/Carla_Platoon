import time
# carriage return print
class c_print:
    def __init__(self):
        self.string = None

    def c_print(self, string):
        if self.string is None:
            self.string = string
            print(string)
        else:
            self.string += "\n" + string
            print(string)

    def clear(self):
        time.sleep(0.001)
        # count \n in self.string
        up = len(self.string.split("\n"))
        for _ in range(up):
            self.UP()
            self.CLR()
        self.string = None

    def UP(self, n = 1):
        print(f"\033[{n}A", end = "")

    def CLR(self):
        print("\033[K", end = "")


if __name__ == "__main__":
    import time
    cprint = c_print()
    for i in range(10):
        cprint.c_print(f"this is line {i}")
        cprint.c_print(f"this is line {i}")
        cprint.c_print(f"this is line {i}")
        cprint.clear()
        time.sleep(0.5)