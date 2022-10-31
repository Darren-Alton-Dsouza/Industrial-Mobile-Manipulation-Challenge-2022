#!/usr/bin/env python3

from __future__ import print_function
from operator import ge
from pickle import PicklingError

import sys
from webbrowser import get
import rospy
from immc_msgs.srv import SubmitOrder, PendingOrders
from gazebo_msgs.srv import GetModelState

color_cubes_count=[0,0,0] #r,g,b

def get_order():
    rospy.wait_for_service('/pending_orders')
    try:
        pendingOrders = rospy.ServiceProxy('/pending_orders', PendingOrders)
        orderList = pendingOrders().pending_orders
        orderDict={}
        # print(orderList)
        color_cubes_count=[0,0,0] #r,g,b
        
        for order in orderList:
            id=order.id
            cube_list=order.objects
            cube_name_with_count=[]

            for cube in cube_list:
                # print(cube)

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
            orderDict[id]=[cube_name_with_count,loc]
        
        # print(orderDict)
        # print(cube_name_with_count)


        return orderDict
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_pickup_loc(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    # print(model_name)

    try:
        ms= rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        x=ms(model_name,'').pose.position.x
        # print(x)
        y=ms(model_name,'').pose.position.y

        pickup_loc=(x,y)
        return pickup_loc

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    # print("hi")
    orders=get_order()
    # for k,v in orders.items():
    #     for c in v[0]:
    #         print(get_pickup_loc(c))
    # print("==============BEFORE=================")
    print(orders)
    order_ids=list(orders.keys())
    # print(order_ids)

    current_id=order_ids.pop(0)

    current_val=orders.pop(current_id)


    # print(current_id)
    print(current_val)
    
    print("==============AFTER=================")
    print(orders)
    # print(order_ids)

    
