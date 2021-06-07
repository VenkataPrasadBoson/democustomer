#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import can
import sys
from numpy import interp
from time import sleep
import logging




# paramers for connecting with can , make sure you provided correctly

    
bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=250000)
# bus = can.interface.Bus(bustype=bus_type, channel=channel, bitrate=baud_rate)


print("---------------------------------------------------------------")

print("         CONNECTION TO CAN DEVICE-- SUCCESS")
print("---------------------------------------------------------------")





def callback(data):
    x_linear=data.linear.x
    x_angular=data.angular.z
    rospy.loginfo("twist.linear: %f ; angular %f", data.linear.x, data.angular.z)
    a=abs(x_linear)
    c=x_angular
    if a==0:
        b=0
    
    elif a>0 or a<0:
        b = a*5
    
    print (b)

    
    #convert to HEX for throttle on little indian 

    hex_v=int(b*256)
    hex_v=hex(hex_v)
    hex_v=hex_v[2:]
    k=hex_v
    if len(k)==4:
        d1=k[:2]
        d2=k[2:]
    elif len(k)==3:
        d1='0'+k[0]
        d2=k[1:]
    elif len(k)==2:
        d1='00'
        d2=k
    elif len(k)==1:
        d1='00'
        d2='0'+k
    else:
        d1='00'
        d2='00'
    d1 = int(d1, 16)
    d2 = int(d2, 16)

    if x_linear>0:
        d3=0X1
    elif x_linear<0:
        d3=0X2
    else: 
        d3=0X0
    print(d1,d2,d3)

    
    msg=[d2,d1,d3,0X0,0X0,0X0,0X0,0X0]
    can_msg = can.Message(arbitration_id=0x510,
                              data=msg,
                              extended_id=False)
    print(msg)
    bus.send(can_msg)


                


def listener():
    rospy.init_node('mobile_subscriber', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

# def can_send_recv():
        
#     bus.send(msg1)
#     autoModeRequestAccepted = False
#     while True:
#         message=bus.recv()
#         msg=message.data
#         if message.arbitration_id==0x501:
#             if msg[2]==0x01:
#                 bus.send(msg3)
#                 autoModeRequestAccepted = True

#             elif msg[2]==0x04 and autoModeRequestAccepted:
#                 autoModeRequestAccepted = False
#                 listener()

#             elif msg[2]==0x03:
#                 autoModeRequestAccepted = False
#                 stop()



               

# def stop():        
#     mes5=bus.recv()
#     if mes5.arbitration_id==0x501:
#         if mes5.data[2]==0x03:
#             sleep(1)
#             can_send_recv()    
                    
        # else:
        #     print("message ")
            # while True:
            #     mess= bus.recv()
            #     if mess.arbitration_id==0x501:
            #         if mess.data[2]==0x03:
            #             print ('ramp')
#     while True:
#         message=bus.recv()
#         msg=message.data
#         if message.arbitration_id==0x501:
#             if msg[2]==0x01:
#                 bus.send(msg3)
#     while True:
#         message1=bus.recv()
#         msg=message1.data
#         if message1.arbitration_id==0x501:
#             if msg[2]==0x04:
#                 break
#                 #listener()  
#         else:
#             print("message ")
#     while True:
#         mess= bus.recv()
#         if mess.arbitration_id==0x501:
#             if mess.data[2]==0x03:
#                 can_send_recv()
# def again(): 
#     if messg.arbitration_id==0x501:
#         if msg[2]==0x03:
#             can_send_recv()

if __name__ == '__main__':
    # can_send_recv()
    listener()
    
    
