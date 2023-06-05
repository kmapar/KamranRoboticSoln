#!/usr/bin/env python3
import sys
import os

import rclpy
import numpy as np
import socket
import time
import sensor

from rclpy.node import Node
from service_pkg.srv import CustomService


class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.service = self.create_service(CustomService, '/sensor_data',self.service_callback)
        self.get_logger().info('Service server initialized.')

    def service_callback(self, request, response):

        # Due to connection issues with service client and sensor.py...
        # num_of_samples has not been optimized
        num_of_samples = 10
    
        # Empty request from service client
        if request is not None:
            num_of_samples = request.num_samples

        # Read data from sensor
        sensor_data = self.read_data(num_of_samples)

        # Filter sensor data
        filtered_data = self.filter(sensor_data)

        # Fill response message with filtered data
        response.filtered_data = filtered_data

        self.get_logger().info('Service client requested data')
        self.get_logger().info('Send response: %s' % response.filtered_data)

        return response
    
    # Read data
    def read_data(self, num_of_samples):
    
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try: 
            # Connect the socket to the port where the server is listening
            server_address = ('127.0.0.1', 10000)

            print('connecting to {} port {}'.format(*server_address))
            sock.connect(server_address)

            while True:
                # Request 'num_of_samples' from the sensor
                message_string = str(num_of_samples)
                message = message_string.encode()
                sock.sendall(message)

                # Delay of 1ms between each sample
                time.sleep(0.001)

                byte_data = sock.recv(10000)
                data = np.frombuffer(byte_data)

                return data
            
        finally:
            # Clean up connection
            self.get_logger().info('Closing socket')
            sock.close()

    # Median filter onto sensor data
    def filter(self, sensor_data):
        window_size = 3

        filtered_data = np.zeros_like(sensor_data)

        for i in range(sensor_data.shape[0]):
            for j in range(sensor_data.shape[1]):
                index = sensor_data[i,j]

                # Neighborhood boundaries set up
                i_start = max(0, i-1)
                i_end = min(i + window_size, sensor_data.shape[0])

                j_start = max(0, j-1)
                j_end = min(j + window_size, sensor_data.shape[1])

                neighborhood = sensor_data[i_start:i_end, j_start:j_end]
                median = np.median(neighborhood)
                filtered_data[i,j] = median

        return filtered_data

    
def main (args = None):
    rclpy.init(args = args)
    
    server = ServiceServer()
    
    rclpy.spin(server)

    # Clean up
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    