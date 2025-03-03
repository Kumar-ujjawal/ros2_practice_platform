#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from factorial_service.srv import Factorial
import sys

class FactorialClient(Node):
    def __init__(self):
        super().__init__('factorial_client')
        self.client = self.create_client(Factorial, 'calculate_factorial')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = Factorial.Request()

    def send_request(self, number):
        self.req.number = number
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result

def main():
    rclpy.init()
    client = FactorialClient()

    # Get the number from command-line argument
    if len(sys.argv) < 2:
        print("Usage: ros2 run factorial_service factorial_client <number>")
        return

    try:
        number = int(sys.argv[1])
    except ValueError:
        print("Please enter a valid integer.")
        return
    
    result = client.send_request(number)
    print(f"Factorial of {number} is {result}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
