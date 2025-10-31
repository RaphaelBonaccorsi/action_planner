#!/usr/bin/env python3

import os, sys, rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
package_share_path = get_package_share_directory("action_planner")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from action_executor_base import ActionExecutorBase
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState


import rclpy


class ActionNodeExample(ActionExecutorBase):

    def __init__(self):
        super().__init__("entrar")
        self.get_logger().info("ActionNodeExample initialized")

    def on_configure_extension(self):
        self.is_action_running = False

        self._can_receive_new_goal = True

        return TransitionCallbackReturn.SUCCESS
    

    def new_goal(self, goal_request) -> bool:
        # Parametros da ação em goal_request.parameters
        passageiro = goal_request.parameters[0]
        andar = goal_request.parameters[1]
        elevador = goal_request.parameters[2]
        self.get_logger().info(f"Passageiro: {passageiro}, Andar: {andar}, Elevador: {elevador}")

        if not self._can_receive_new_goal:
            self.get_logger().info("Cannot receive new goal, action is already running")
            return False
        
        self.get_logger().info(f"New goal received {goal_request.parameters}")
        self.progress_ = 0.0

        self._can_receive_new_goal = False
        self.is_action_running = True
        
        return True

    def execute_goal(self, goal_handle):

        self.progress_ = 1.0  # Simulate immediate completion for this example

        if self.progress_ < 1.0:
            return False, self.progress_
        else:
            self._can_receive_new_goal = True
            self.is_action_running = False
            return True, 1.0

    def cancel_goal(self, goal_handle):
        self.get_logger().info("Canceling goal")
        self.is_action_running = False
        self._can_receive_new_goal = True

    def cancel_goal_request(self, goal_handle):
        self.get_logger().info("Cancel goal request received")
        return True
    

    ###################################################


def main(args=None):
    rclpy.init(args=args)
    node = ActionNodeExample()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()