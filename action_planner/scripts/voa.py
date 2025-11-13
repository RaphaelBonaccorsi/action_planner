#!/usr/bin/env python3

import os, sys, rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
package_share_path = get_package_share_directory("action_planner")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)

from action_executor_base import ActionExecutorBase
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from harpia_msgs.srv import PlanPath


import rclpy


class ActionNodeExample(ActionExecutorBase):

    def __init__(self):
        super().__init__("voa")
        self.get_logger().info("ActionNodeExample initialized")

    def on_configure_extension(self):
        self.is_action_running = False
        self._can_receive_new_goal = True
        
        # Create service client for path planning
        self.path_planning_client = self.create_client(PlanPath, 'path_planner/plan_path')
        
        return TransitionCallbackReturn.SUCCESS
    

    def new_goal(self, goal_request) -> bool:
        # Parametros da ação em goal_request.parameters
        drone = goal_request.parameters[0]
        origem = goal_request.parameters[1]
        destino = goal_request.parameters[2]
        self.get_logger().info(f"Voando\nDrone: {drone}, Origem: {origem}, Destino: {destino}")

        if not self._can_receive_new_goal:
            self.get_logger().info("Cannot receive new goal, action is already running")
            return False
        
        self.get_logger().info(f"New goal received {goal_request.parameters}")
        
        # Request path planning
        self.get_logger().info(f"Requesting path planning from {origem} to {destino}...")
        
        # Wait for service to be available
        if not self.path_planning_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Path planning service not available!")
            return False
        
        # Create and send request (synchronous call)
        request = PlanPath.Request()
        request.origem = origem
        request.destino = destino
        
        try:
            # Use synchronous call to avoid blocking the executor
            response = self.path_planning_client.call(request)
            
            if response.success:
                self.get_logger().info(f"Path planning successful!")
                self.get_logger().info(f"Waypoints: {response.waypoints}")
                self.get_logger().info(f"Message: {response.message}")
                self.planned_waypoints = response.waypoints
            else:
                self.get_logger().error(f"Path planning failed: {response.message}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Path planning service call failed: {e}")
            return False
        
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