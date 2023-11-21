import socket
import json
import time

# this udp sever is only used for processing IMU and GNSS data
# if you want to have more functions, you should modify this class
class udp_server():
    def __init__(self):
        self.dest_host = 'localhost'
        self.dest_port = 9870
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.imu_gnss_data = {
            "timestamp": 0.0,
            "IMU": {
                "accelerometer": [0.0, 0.0, 0.0],
                "gyroscope": [0.0, 0.0, 0.0]
            },
            "GNSS": {
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0
            },
            "throttle": 0.0,
            "brake": 0.0,
            "steer": 0.0,
        }

    def update_throttle(self, throttle):
        self.imu_gnss_data['throttle'] = throttle
    
    def update_brake(self, brake):
        self.imu_gnss_data['brake'] = brake
    
    def update_steer(self, steer):
        self.imu_gnss_data['steer'] = steer

    def update_IMU(self, acc_gyro):
        acc = acc_gyro[0]
        gyro = acc_gyro[1]
        self.imu_gnss_data['IMU']['accelerometer'] = acc
        self.imu_gnss_data['IMU']['gyroscope'] = gyro
    
    def update_GNSS(self, gnss):
        self.imu_gnss_data['GNSS']['latitude'] = gnss[0]
        self.imu_gnss_data['GNSS']['longitude'] = gnss[1]
        self.imu_gnss_data['GNSS']['altitude'] = gnss[2]

    def update(self):
        # update timestamp
        self.imu_gnss_data['timestamp'] = time.time()
        # send data
        self.sock.sendto(json.dumps(self.imu_gnss_data).encode(), (self.dest_host, self.dest_port))

        
