#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from harpia_msgs.srv import StrInOut
from harpia_msgs.action import ExecutePlan
from std_msgs.msg import String
import json

class MissionController(LifecycleNode):
    def __init__(self):
        super().__init__('mission_controller')
        self.update_parameters_client = None
        self.logger = self.get_logger()
        
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Configuring...')

        self.isMissionRunning = False
        self.isWaitingCancelation = False

        self.update_parameters_client = self.create_client(StrInOut, 'plansys_interface/update_parameters')
        self.goal_handle = None
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Activating...')
        
        # Ensure service is available
        if not self.update_parameters_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('Service not available, activation failed!')
            return TransitionCallbackReturn.ERROR
        
        self.execute_plan_action_client = ActionClient(self, ExecutePlan, 'action_planner/execute_plan')

        self.requestPlanExecution()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Cleaning up...')
        self.destroy_client(self.update_parameters_client)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info('Shutting down...')
        self.destroy_client(self.update_parameters_client)
        return TransitionCallbackReturn.SUCCESS
    
    def send_parameters_update(self, updates, callback_func):

        # Create and send request
        request = StrInOut.Request()
        request.message = json.dumps(updates)

        # self.get_logger().info(f'@@ SENDING PARAMETERS UPDATE')
            
        future = self.update_parameters_client.call_async(request)
        future.add_done_callback(callback_func)


    def requestPlanExecution(self):

        if self.isMissionRunning:
            self.get_logger().error('Error: Cant start a plan, another is running')
            return
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.execute_plan_action_client.wait_for_server()

        # Create goal
        goal_msg = ExecutePlan.Goal()

        # Send goal and register callbacks
        self._send_goal_future = self.execute_plan_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
            
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.isMissionRunning = True

        # Get result
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Mission ended, final result: {result.success}')
        self.isMissionRunning = False

        if self.isWaitingCancelation:
            self.isWaitingCancelation = False
            self.get_logger().info('Mission canceled.')
            
            if hasattr(self, 'whatToRunWhenFinishedCancelation'):
                self.whatToRunWhenFinishedCancelation(True)
                del self.whatToRunWhenFinishedCancelation

            # if self.isWaitingToReplan:
            #     self.isWaitingToReplan = False
            #     self.get_logger().info('Now replanning...')
            #     if self.whatToRunWhenReplan is None:
            #         self.get_logger().error('No replan function defined')
            #     else:
            #         self.whatToRunWhenReplan()
            #         self.whatToRunWhenReplan = None

        else:
            if result.success:
                self.get_logger().info('Mission completed successfully')
            else:
                self.get_logger().error('Mission failed, lets try again')
                self.requestPlanExecution()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: Step {feedback.step}/{feedback.nofsteps}')

        # if feedback.step == 2:
        #     self.cancel_goal()

    def request_cancel_goal(self, func_callback=None):

        if not self.goal_handle:
            self.get_logger().error('No goal to cancel')
            func_callback(False)
            return
        
        self.get_logger().info('Requesting goal cancellation...')

        if self.isWaitingCancelation:
            self.get_logger().error('ERROR, cancelation request already sent')
            func_callback(False)
            return
        
        if func_callback is not None:
            self.whatToRunWhenFinishedCancelation = func_callback

        def cancel_response_callback(future):
            cancel_response = future.result()
            if cancel_response.return_code == 0:  # GOAL_TERMINAL_STATE
                self.get_logger().info('Cancellation request succeeded')
                self.isWaitingCancelation = True
            else:
                self.get_logger().warn(f'Cancellation request failed with code: {cancel_response.return_code}')

                if hasattr(self, 'whatToRunWhenFinishedCancelation'):
                    self.whatToRunWhenFinishedCancelation(False)
                    del self.whatToRunWhenFinishedCancelation
                # if self.isWaitingToReplan:
                #     self.get_logger().error("could not replan, cancelation failed")
                #     self.isWaitingToReplan = False
                #     self.whatToRunWhenReplan = None

        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(cancel_response_callback)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = MissionController()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()