from march_shared_msgs.srv import SolveInverseKinematic
import rclpy
from rclpy.node import Node
import time
import random

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("ik_solver_client")
        self.cli = self.create_client(
            SolveInverseKinematic, "/rl/inverse_kinematic_solver", 
        )
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("service not available, waiting again...")
        self.get_logger().info("connected with service!")
        self.req = SolveInverseKinematic.Request()

    def send_request(self, pos_forward, pos_upward):
        self.req.desired_location.x = float(0)
        self.req.desired_location.y = float(pos_forward)
        self.req.desired_location.z = float(pos_upward)
        self.future = self.cli.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)

    start = time.time()
    minimal_client = MinimalClientAsync()
    minimal_client.send_request(random.randint(0, 30), random.randint(0,20))
    end = time.time()

    print("\n This took ", end-start, "seconds")

    # minimal_client.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
