import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class ActionClientManager(Node):
    def __init__(self, client_name, action_type, action_name):
        super().__init__(client_name)
        self._action_client = ActionClient(self, action_type, action_name)
        self.action = action_type
        self.order = None

    def make_order(self):
        pass

    def send_goal(self, order):
        goal_msg = self.action.Goal()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    #ver como se comporta el sequence y partial sequence (hardcodeado)
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

class ClientAsync(Node):

    def __init__(self, client_name, interface_type, service_name):
        super().__init__(client_name)
        self.cli = self.create_client(interface_type, service_name)
        self.req = interface_type.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)