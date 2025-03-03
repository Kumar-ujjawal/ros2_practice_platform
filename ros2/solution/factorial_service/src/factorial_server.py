#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from factorial_service.srv import Factorial

class FactorialService(Node):
    def __init__(self):
        super().__init__('factorial_server')
        self.srv = self.create_service(Factorial, 'calculate_factorial', self.factorial_callback)
        self.get_logger().info("Factorial Service is Ready!")

    def factorial_callback(self, request, response):
        num = request.number
        response.result = self.calculate_factorial(num)
        self.get_logger().info(f"Received request for {num}! -> {response.result}")
        return response

    def calculate_factorial(self, n):
        if n < 0:
            return -1  # Error case (negative number)
        result = 1
        for i in range(2, n + 1):
            result *= i
        return result

def main():
    rclpy.init()
    node = FactorialService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
