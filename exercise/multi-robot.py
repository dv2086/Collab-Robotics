import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amazon_robot_msg.action import FollowTargets
from geometry_msgs.msg import PoseStamped
import numpy as np

'''
JdeMultirobot 

The goal of this exercise is to create a plan which describes the pickup and drop-off locations of pallets, and then
executes it. A plan for each robot consists of two arrays describing locations to drive to (PoseStamped)
and action to be performed (Load /unload/ do nothing).

In our use case, task is described as 
[drive to pickup -> picking up -> drive to drop-off -> unload] 

Following are some of the mandatory (!) , optional (/) and feature(*) goals. For feature goals, you will also have 
to implement the logic code by yourself. 

1. (!) Create a collision Free plan. No robots should collide with each other.
2. (!) Move at least 2 pallets from pallet locations to storage location
3. (/) No replanning / Recovery behaviour used while driving
4. (/) Fastest moving of all pallets from pickup to drop-off
5. (/) Fastest moving of all pallets from pickup to drop-off and back
6. (/) Lowest average time spent per pallet per task. One task is drive -> pickup -> drop-off
7. (/) Avoiding cross paths
8. (/) Multiple stops before reaching goal
9. (/) Dynamic replanning based on current conditions 
10. (*) Energy of the robot required to drive to the pallet
11. (*) High priority pallet first.

and so on .. 

You can consider following parameters while making the plan:
1. Distance from robot to the nearest pallet
2. Distance of pallet from the storage location
3. Current waypoints, remaining waypoints ..
4. State of the lift joint  

To access these parameters, you will have to use various ros concepts such as subscribing to a topic or 
creating an action client (just like this one!)

For example, to get a path between current position of the robot and another position (in our case, a pallet
at positon (x,y,z)), you can call the action server /<robot_name>/compute_path_to_pose which will return you 
an array of points of the path.

You can also get the current robot position using robot odometry topic (/<robot_name>/odom), which you can use for
different calculations. You can also experiment with different path planners, costmaps, behaviour trees and planners.


Following topics might be helpful:
/<robot_name>/amcl_pose
/<robot_name>/scan
/<robot_name>/imu
/<robot_name>/odom
/<robot_name>/initialpose
/<robot_name>/cmd_vel
/<robot_name>/map


Following action servers might be useful:
/<robot_name>/FollowTargets
/<robot_name>/backup
/<robot_name>/compute_path_to_pose
/<robot_name>/follow_path
/<robot_name>/navigate_to_pose
/<robot_name>/spin
/<robot_name>/wait


Have fun!

'''


pallets = {
    'pallet_1': {'x_pose': 3.64, 'y_pose': 0.63, 'z_pose': 0.01, 'rotation': 3.14},
    'pallet_2': {'x_pose': 3.59, 'y_pose': -1.11, 'z_pose': 0.01, 'rotation': 3.14},
    'pallet_3': {'x_pose': 3.51, 'y_pose': -2.84, 'z_pose': 0.01, 'rotation': 1.5708},
    'pallet_4': {'x_pose': 3.49, 'y_pose': -4.68, 'z_pose': 0.01, 'rotation': 1.5708},
    'pallet_5': {'x_pose': 3.64, 'y_pose': -6.91, 'z_pose': 0.01, 'rotation': -2.108},
    'pallet_6': {'x_pose': 3.64, 'y_pose': -8.88, 'z_pose': 0.01, 'rotation': -1.5708},
}

storage_locations = {
    'storage_location_0': {'x_pose': -5.84, 'y_pose': -7.76, 'z_pose': 0.01, 'rotation': 3.14},
    'storage_location_1': {'x_pose': -5.84, 'y_pose': -3.35, 'z_pose': 0.01, 'rotation': 3.14},
    'storage_location_2': {'x_pose': -5.84, 'y_pose': 1.12, 'z_pose': 0.01, 'rotation': 3.14},
    'storage_location_3': {'x_pose': -5.84, 'y_pose': 6.12, 'z_pose': 0.01, 'rotation': 3.14},
}

middlepoint = {
    'preentrance_left': {'x_pose': 0.97, 'y_pose': 2.3, 'z_pose': 0.01, 'rotation': -1.5708},
    'entrance_left': {'x_pose': 3.58, 'y_pose': 2.3, 'z_pose': 0.01, 'rotation': 0},

    'preentrance_right1': {'x_pose': 0.97, 'y_pose': -6.91, 'z_pose': 0.01, 'rotation': -1.5708},
    'preentrance_right2': {'x_pose': 0.97, 'y_pose': -9.88, 'z_pose': 0.01, 'rotation': -1.5708},
    'entrance_right': {'x_pose': 3.58, 'y_pose': -9.88, 'z_pose': 0.01, 'rotation': 0},

    'exit1': {'x_pose': 0.97, 'y_pose': 0.49, 'z_pose': 0.01, 'rotation': 3.14},
    'exit2': {'x_pose': 0.97, 'y_pose': -1.56, 'z_pose': 0.01, 'rotation': 3.14},

    'exit5': {'x_pose': 0.7, 'y_pose': -8.0, 'z_pose': 0.01, 'rotation': 1.57},
    'exit6': {'x_pose': 0.7, 'y_pose': -9.5, 'z_pose': 0.01, 'rotation': 3.14},
    'middle_right_down': {'x_pose': 0.7, 'y_pose': -5.7, 'z_pose': 0.01, 'rotation': 3.14},
    'middle_right_up': {'x_pose': 0.7, 'y_pose': -5.7, 'z_pose': 0.01, 'rotation': 0} ,

    'middle_left_down': {'x_pose': 0.9, 'y_pose': 0.63, 'z_pose': 0.01, 'rotation': 3.14},
    'middle_left_up': {'x_pose': 0.9, 'y_pose': 0.63, 'z_pose': 0.01, 'rotation': 0},

    'storage_entrance0': {'x_pose': -3.5, 'y_pose': -7.7, 'z_pose': 0.01, 'rotation': 3.14},
    'storage_entrance1': {'x_pose': -3.5, 'y_pose': -3.35, 'z_pose': 0.01, 'rotation': 3.14},
    'storage_entrance2': {'x_pose': -4.24, 'y_pose': 1.12, 'z_pose': 0.01, 'rotation': 3.14},
    'storage_entrance3': {'x_pose': -4.24, 'y_pose': 6.12, 'z_pose': 0.01, 'rotation': 3.14},

    'bottom_left_down': {'x_pose': -4.24, 'y_pose': 0.63, 'z_pose': 0.01, 'rotation': 3.14},
    'bottom_right_down': {'x_pose': -3.5, 'y_pose': -5.7, 'z_pose': 0.01, 'rotation': 3.14}
}    


free_area = {
    'right_end_of_corridor': {'x_pose': 1.35, 'y_pose': -6.78, 'z_pose': 0.01, 'rotation': 0},
    'left_end_of_corridor': {'x_pose': 0.92, 'y_pose': 6.45, 'z_pose': 0.01, 'rotation': 0},
}

lift_stages = {'load': 2, 'unload': -2, 'pload': 4, 'half_load': 1, 'half_unload': -1, 'punload': -4, 'unchanged': 0}

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

def get_pose_stamped(x_pose, y_pose, z_pose, rotation):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose
    q = quaternion_from_euler(0, 0, rotation)
    pose_stamped.pose.orientation.x = q['x']
    pose_stamped.pose.orientation.y = q['y']
    pose_stamped.pose.orientation.z = q['z']
    pose_stamped.pose.orientation.w = q['w']
    return pose_stamped


class WarehouseController:

    def __init__(self):
        self.action_client1 = FollowTargetActionClient('/robot1/FollowTargets', 'robot1_follow_target_action_client')
        self.action_client2 = FollowTargetActionClient('/robot2/FollowTargets', 'robot2_follow_target_action_client')


    # TODO: Use this function to create your plan
    def create_plan(self):        
        poses_robot1 = [ get_pose_stamped(**middlepoint['middle_right_up']),
                        get_pose_stamped(**middlepoint['preentrance_right1']),
                        get_pose_stamped(**middlepoint['preentrance_right2']),
                        get_pose_stamped(**middlepoint['entrance_right']),
                        get_pose_stamped(**pallets['pallet_6']),

                        get_pose_stamped(**middlepoint['exit6']),
                        get_pose_stamped(**middlepoint['middle_right_down']),
                        get_pose_stamped(**middlepoint['bottom_right_down']),
                        get_pose_stamped(**middlepoint['storage_entrance0']),
                        get_pose_stamped(**storage_locations['storage_location_0']),
                        
                        get_pose_stamped(**middlepoint['middle_right_up']),
                        get_pose_stamped(**middlepoint['preentrance_right1']),
                        get_pose_stamped(**middlepoint['preentrance_right2']),
                        get_pose_stamped(**middlepoint['entrance_right']),

                        get_pose_stamped(**pallets['pallet_5']),
                        get_pose_stamped(**pallets['pallet_6']),
                        get_pose_stamped(**middlepoint['exit5']),
                        get_pose_stamped(**middlepoint['middle_right_down']),
                        get_pose_stamped(**middlepoint['bottom_right_down']),
                        get_pose_stamped(**storage_locations['storage_location_1']),
                        
                        get_pose_stamped(**middlepoint['middle_right_up']),
                        get_pose_stamped(**free_area['right_end_of_corridor'])]

        loads_robot1 = [lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['pload'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['punload'],
        lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['pload'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['punload'], 
        lift_stages['unchanged'], lift_stages['unchanged']]

        poses_robot2 = [get_pose_stamped(**middlepoint['preentrance_left']),
                        get_pose_stamped(**middlepoint['entrance_left']),
                        get_pose_stamped(**pallets['pallet_1']),
                        get_pose_stamped(**middlepoint['exit1']),
                        get_pose_stamped(**middlepoint['middle_left_down']),

                        get_pose_stamped(**middlepoint['storage_entrance3']),
                        get_pose_stamped(**storage_locations['storage_location_3']),

                        get_pose_stamped(**middlepoint['middle_left_up']),
                        get_pose_stamped(**middlepoint['preentrance_left']),
                        get_pose_stamped(**middlepoint['entrance_left']),
                        get_pose_stamped(**pallets['pallet_2']),
                        get_pose_stamped(**middlepoint['middle_left_down']),

                        get_pose_stamped(**middlepoint['storage_entrance2']),
                        get_pose_stamped(**storage_locations['storage_location_2']),

                        get_pose_stamped(**middlepoint['middle_left_up']),
                        get_pose_stamped(**free_area['left_end_of_corridor'])
                        ]



        loads_robot2 = [lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['pload'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'],  lift_stages['punload'],
        lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['unchanged'], lift_stages['pload'], lift_stages['unchanged'], lift_stages['unchanged'],  lift_stages['punload'], lift_stages['unchanged'], lift_stages['unchanged']]


        print("Sending target goals to robot1 and robot2")
        self.action_client1.send_targets(poses_robot1, loads_robot1)
        self.action_client2.send_targets(poses_robot2, loads_robot2)

    # def create_test_plan(self):
    #     poses_robot1 = [get_pose_stamped(**pallets['pallet_1']),
    #                     get_pose_stamped(**storage_locations['storage_location_1']),
    #                     get_pose_stamped(**pallets['pallet_3']),
    #                     get_pose_stamped(**storage_locations['storage_location_3'])
    #                     ]

    #     loads_robot1 = [lift_stages['load'], lift_stages['unload'], lift_stages['load'], lift_stages['unload']]
    #     poses_robot2 = [get_pose_stamped(**free_area['right_end_of_corridor']),
    #                     get_pose_stamped(**pallets['pallet_2']),
    #                     get_pose_stamped(**storage_locations['storage_location_2']),
    #                     get_pose_stamped(**pallets['pallet_4']),
    #                     get_pose_stamped(**pallets['pallet_1'])]

    #     loads_robot2 = [lift_stages['unchanged'], lift_stages['load'], lift_stages['unload'], lift_stages['load'], lift_stages['unload']]
    #     #
    #     # print("Poses List ")
    #     # print(poses_robot1)
    #     # print("Load List ")
    #     # print(loads_robot1)
    #     print("Sending target goals to robot1 and robot2")

    #     self.action_client1.send_targets(poses_robot1, loads_robot1)
    #     self.action_client2.send_targets(poses_robot2, loads_robot2)

    def execute_plan(self):
        rclpy.spin(self.action_client1)
        rclpy.spin(self.action_client2)


class FollowTargetActionClient(Node):

    def __init__(self, action_name='/robot1/FollowTargets', action_client_name='follow_target_action_client'):
        super().__init__(action_client_name)
        self._action_client = ActionClient(self, FollowTargets, action_name)


    def get_action_client(self):
        return self._action_client

    def send_targets(self, poses, loads):
        self.get_logger().info('Received Goal poses: {0}'.format(len(poses)))
        self.get_logger().info('Received Load instructions: {0}'.format(len(loads)))
        if len(poses) == len(loads):
            goal_msg = FollowTargets.Goal()
            goal_msg.poses = poses
            goal_msg.load = loads
            self.goal_length = len(poses)
            print(self._action_client)
            self._action_client.wait_for_server()
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        else:
            self.get_logger().warn('Unequal amount of Poses and loads!')


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Currently Executing: {0} out of {0} targets'.format(feedback.current_waypoint, self.goal_length))



def main(args=None):
    rclpy.init(args=args)
    warehouse_controller = WarehouseController()
    warehouse_controller.create_plan()
    #warehouse_controller.create_test_plan()
    warehouse_controller.execute_plan()


if __name__ == '__main__':
    main()
