#!/usr/bin/env python3

from time import sleep
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
from immc_msgs.srv import SubmitOrder, PendingOrders
from gazebo_msgs.srv import GetModelState
import sys
import moveit_commander
import moveit_msgs.msg
from gazebo_msgs.msg import ContactsState
from gazebo_ros_link_attacher.srv import Attach
from std_srvs.srv import Empty


pick_up_location = (7.725, -8.355)


class IMMC():

    def __init__(self):
        self.current_job=0
        self.color_cubes_count=[0,0,0] #r,g,b
        self.orderDict={}
        self.pickup_loc=(0,0)
        self.distination_loc=(0,0)
        moveit_commander.roscpp_initialize(sys.argv)
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.gripper_client= actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)
        self.robot1_group = moveit_commander.MoveGroupCommander("arm")
        self.robot1_client = actionlib.SimpleActionClient('execute_trajectory',moveit_msgs.msg.ExecuteTrajectoryAction)


    def get_order(self):
        rospy.wait_for_service('/pending_orders')
        try:
            pendingOrders = rospy.ServiceProxy('/pending_orders', PendingOrders)
            orderList = pendingOrders().pending_orders
            # print(orderList)
            color_cubes_count=[0,0,0] #r,g,b
            
            for order in orderList:
                id=order.id
                cube_list=order.objects
                cube_name_with_count=[]

                for cube in cube_list:
                    if cube[0] == 'r':
                        cube_name_with_count.append(cube+str(color_cubes_count[0]))
                        color_cubes_count[0]+=1
                    elif cube[0] == 'g':
                        cube_name_with_count.append(cube+str(color_cubes_count[1]))
                        color_cubes_count[1]+=1    
                    elif cube[0] == 'b':
                        cube_name_with_count.append(cube+str(color_cubes_count[2]))
                        color_cubes_count[2]+=1


                x=order.desired_location.x
                y=order.desired_location.y
                loc=(x,y)
                self.orderDict[id]=[cube_name_with_count,loc]
            
            # print(self.orderDict)
            # print(cube_name_with_count)
            return self.orderDict
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    
    def get_pickup_loc(self,model_name):
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            ms= rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            x=ms(model_name,'').pose.position.x
            y=ms(model_name,'').pose.position.y

            self.pickup_loc=(x,y)
            return self.pickup_loc

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def assign_job(self,job_num):
        self.current_job=job_num # get order = 0, go to collection pose = 1, pick the cube = 2, go to drop pose  = 3, drop cube = 4, check order = 5, completed = 6
        return self.current_job

    def movebase_client(self,x_coord, y_coord):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_coord
        goal.target_pose.pose.position.y = y_coord
        quat = tf.transformations.quaternion_from_euler(0,0,3.14159)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()


    def attach_cube_to_gripper(self,object_name,contact=None):
        rospy.wait_for_service('/link_attacher_node/attach')
        attach = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

        # result = attach.call(contact.collision1_name.split("::")[0], contact.collision1_name.split("::")[1], contact.collision2_name.split("::")[0], contact.collision2_name.split("::")[1])
        result = attach.call(object_name, 'base_link', 'robot', 'gripper_link')
        result = attach.call(object_name, 'base_link', 'robot', 'gripper_link_sub')
        return result

    def get_contacts(self,object_name,contacts_message=None):
        if (len(contacts_message.states) != 0):
            if ('gripper_link' or 'gripper_link_sub' in contacts_message.states[0].collision1_name) :
                rospy.loginfo("Collision detected with %s." % contacts_message.states[0].collision2_name.split("::")[0])
                self.attach_cube_to_gripper(object_name)
            elif ('gripper_link' or 'gripper_link_sub' in contacts_message.states[0].collision2_name) :
                rospy.loginfo("Collision detected with %s." % contacts_message.states[0].collision1_name.split("::")[0])
                self.attach_cube_to_gripper(object_name)
            else:
                rospy.loginfo("Unknown collision")


    def move_gripper(self,target_position):
        
        self.gripper_client.wait_for_server()
        rospy.loginfo('Execute Trajectory server is available for gripper')
        self.gripper_group.set_named_target(target_position)
        _, plan, _, _ = self.gripper_group.plan()
        gripper_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        gripper_goal.trajectory = plan
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result()

    def move_arm(self,target_position):
        
        self.robot1_client.wait_for_server()
        rospy.loginfo('Execute Trajectory server is available for arm')
        self.robot1_group.set_named_target(target_position)
        _, plan, _, _ = self.robot1_group.plan()
        robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        robot1_goal.trajectory = plan
        self.robot1_client.send_goal(robot1_goal)
        self.robot1_client.wait_for_result()

    def pick_object(self,object_name):
        # moveit_commander.roscpp_initialize(sys.argv)
        # print("Inside Pick Object")
        #open gripper
        self.move_gripper("full_open")
        rospy.loginfo("Gripper moved to 'full_open'")

        #move the arm to the pre_grasp position
        self.move_arm("pre_grasp")
        rospy.loginfo("Arm moved to 'pre_grasp'")

        
        #close the gripper (grasp the object)
        self.move_gripper("close")
        rospy.loginfo("Gripper moved to 'close'")

        
        #fix grasping (attach cube to the gripper)
        try: 
            # contact_msg = rospy.wait_for_message('/gripper_contacts', ContactsState)
            # self.get_contacts(contact_msg)
            rospy.loginfo("Gripping the cube:{}".format(object_name))
            self.attach_cube_to_gripper(object_name)
        except:
            rospy.logwarn("grasp fix failed")

        #move the arm to the transportation position
        self.move_arm("transportation")


    def drop_object(self,object_name):
        self.move_arm("pre_release")

        detach = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        res = detach.call(object_name, 'base_link', 'robot', 'gripper_link')
        res = detach.call(object_name, 'base_link', 'robot', 'gripper_link_sub')
        
        self.move_gripper("full_open")

        self.move_arm("home")
        
        # When finished shut down moveit_commander.
        # moveit_commander.roscpp_shutdown()

    def clear_costmaps(self):
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_cm = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
            rospy.loginfo("Clearing Costmaps")
            clear_cm()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



def main():
    rospy.init_node('mobile_arm', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    immc=IMMC()
    current_job=immc.assign_job(0)
    rospy.loginfo_once(current_job)

    rospy.loginfo("Getting Order")
    orderDict=immc.get_order()
    order_ids=list(orderDict.keys())

    while not rospy.is_shutdown() and current_job is not immc.assign_job(6): 
        if current_job == immc.assign_job(0):
            if len(order_ids):
                current_id=order_ids.pop(0)
                current_val=orderDict.pop(current_id)
                current_job = immc.assign_job(1)
            else:
                rospy.loginfo("Finishing All Orders")
                rospy.logwarn("END OF TASKS")
                current_job = immc.assign_job(6)

        elif current_job == immc.assign_job(1):
            # rospy.loginfo("Going to Pickup Point")
            if len(current_val[0])>1:
                print("Multiple delivery in an ID")
            current_sub_task=current_val[0].pop(0)
            rospy.loginfo("Current Cube:{}".format(current_sub_task))
            immc.get_pickup_loc(current_sub_task)

            # Setting up the pickup location
            robot_stading_offset=(0.17,0)
            rospy.loginfo("Going to Pickup Point: ({},{})".format(immc.pickup_loc[0]+robot_stading_offset[0],immc.pickup_loc[1]+robot_stading_offset[1]))
            immc.movebase_client(immc.pickup_loc[0]+robot_stading_offset[0],immc.pickup_loc[1]+robot_stading_offset[1])

            current_job = immc.assign_job(2)

        elif current_job == immc.assign_job(2):
            rospy.loginfo("Grabbing object")
            immc.pick_object(current_sub_task)
            current_job = immc.assign_job(3)
            rospy.loginfo("Assigning job_{} to go".format(current_job))

        elif current_job == immc.assign_job(3):
            # clear costmaps
            immc.clear_costmaps()

            x_goal = current_val[1][0]
            y_goal = current_val[1][1]

            x_inter=7.6
            y_inter=-6.4

            # Setting intermediate point to avoid collisions
            rospy.loginfo("Going to Intermediate Point ({},{})".format(x_inter,y_inter))
            immc.movebase_client(x_inter,y_inter)
            
            rospy.loginfo("Going to Destination Point: ({},{})".format(x_goal,y_goal))
            immc.movebase_client(x_goal,y_goal)
            current_job = immc.assign_job(4)

        elif current_job == immc.assign_job(4):
            rospy.loginfo("Releasing Object")
            immc.drop_object(current_sub_task)
            current_job = immc.assign_job(5)

        elif current_job == immc.assign_job(5):
            rospy.loginfo("Finishing order")
            if len(current_val[0])== 0:
                print("inside if")
                rospy.wait_for_service("/submit_order")
                submit_order = rospy.ServiceProxy("/submit_order", SubmitOrder)
                order_status = submit_order(current_id) #order_id
                if order_status:
                    rospy.loginfo("Order finished")
                else:
                    rospy.logerr("Order not accepted")
                print("current job:",current_job)

                current_job = immc.assign_job(0)

                
            else:
                current_job = immc.assign_job(1)


        else:
            rospy.logerr("NOT A VALID STATE IN THE STATE MACHINE")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
