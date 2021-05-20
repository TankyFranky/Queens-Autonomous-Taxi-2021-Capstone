#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import String

import math
import time


command_ = None
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'stop',
    1: 'forward',
    2: 'turnright',
    3: 'turnleft',
}

# def clbk_laser(msg):
#     global regions_
#     regions_ = {
#         'right':  min(min(msg.ranges[0:143]), 10),
#         'fright': min(min(msg.ranges[144:287]), 10),
#         'front':  min(min(msg.ranges[288:431]), 10),
#         'fleft':  min(min(msg.ranges[432:575]), 10),
#         'left':   min(min(msg.ranges[576:719]), 10),
#     }
    #userinput = input("Get ur input: 1-left, 2-right")

#advanced region split
def clbk_laser(msg):
    global regions_
    global ir, il
    regions_ = {
            'r1':   min(min(msg.ranges[0:71]), 10),
            'r2':   min(min(msg.ranges[72:144]), 10),
            'r3':   min(min(msg.ranges[145:215]), 10),
            'r4':   min(min(msg.ranges[216:288]), 10),
            'r5':   min(min(msg.ranges[289:359]), 10),
            'l5':   min(min(msg.ranges[360:431]), 10),
            'l4':   min(min(msg.ranges[432:503]), 10),
            'l3':   min(min(msg.ranges[504:576]), 10),
            'l2':   min(min(msg.ranges[577:647]), 10),
            'l1':   min(min(msg.ranges[648:721]), 10),
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
            'r1r':	min(min(msg.ranges[0:20]), 10),
            'r2r':	min(min(msg.ranges[80:100]), 10),
            'rmr':  min(min(msg.ranges[40:60]), 10),
            'l2l':  min(min(msg.ranges[601:621]), 10),
            'l1l':	min(min(msg.ranges[701:721]), 10),
            'lml':	min(min(msg.ranges[641:661]), 10),

            #assuming 721 = pi
    }

    # dr = regions_['r2r'] * 0.73728 - regions_['r1r']
    # dl = regions_['l2l'] * 0.73728 - regions_['l1l']
    # ir = regions_['r1r']/dr
    # il = regions_['l1l']/dl

    
    take_action()

def callback(msg):
    global command_
    command_ = msg.data
    

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():


    global regions_, Aref_r, Aref_l, fku, idist 
    idist = path_width()
    #print(idist)
    fku = 1
    if command_ == 'a'and fku ==1 :
        followleft()
    elif command_ == 'w'and fku ==1:
        straight() 
    elif command_ == 'd'and fku ==1:
        followright()
    elif command_ == 's'and fku ==1:
        change_state(0)
    else:
        change_state(0)

    #print(il, "-", ir)
    
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    

    # print(Aref_l, "-", Aref_r)
    # V is the theoredical value of the distance
    # V= math.sin( Adev) * regions ['right'] / sin(180 - Adev - 72) ##measured region angle
    #add recursion function for every next region for v3.0     

    #state_description = ''
    # d = 1

    # if regions['r1'] > d and regions['r2'] > d and regions['r3'] > d and regions['r4'] > d*1.5 and regions['r5'] > d and regions['l5'] > d and regions['l4'] > d and regions['l3'] < d and regions['l2'] < d and regions['l1'] < d :
    #     state_description = 'case 1 - LTpos'
    #     print ('case 1 - LTpos')
    #     change_state(1)

    # elif regions['r1'] > d and regions['r2'] > d*1.5 and regions['r3'] > d*1.5 and regions['r4'] > d and regions['r5'] > d and regions['l5'] > d and regions['l4'] > d and regions['l3'] < d and regions['l2'] < d and regions['l1'] < d :
    #     state_description = 'case 2 - LTinc'
    #     print ('case 2 - LTinc')
    #     change_state(3)

    # elif regions['r1'] > d*1.5 and regions['r2'] > d*1.5 and regions['r3'] > d and regions['r4'] > d and regions['r5'] > d and regions['l5'] > d and regions['l4'] > d and regions['l3'] < d and regions['l2'] < d and regions['l1'] < d :
    #     state_description = 'case 3 - LTstp'
    #     print ('case 3 - LTstp')
    #     change_state(3)


    # elif regions['r1'] > d and regions['r2'] > d and regions['r3'] > d and regions['r4'] > d and regions['r5'] < d*1.5 and regions['l5'] < d*1.5 and regions['l4'] > d and regions['l3'] > d and regions['l2'] > d and regions['l1'] > d :
    #     state_description = 'case 4 - FWDop'
    #     print ('case 4 - FWDop')
    #     change_state(0)


    # elif regions['r1'] > d and regions['r2'] > d and regions['r3'] > d and regions['r4'] > d and regions['r5'] < d and regions['l5'] < d and regions['l4'] > d and regions['l3'] > d and regions['l2'] > d and regions['l1'] > d :
    #     state_description = 'case 5 - FWDstp'
    #     print ('case 5 - FWDstp')
    #     change_state(1)


    # elif regions['r1'] < d and regions['r2'] < d and regions['r3'] < d and regions['r4'] > d and regions['r5'] > d and regions['l5'] > d and regions['l4'] > d*1.5 and regions['l3'] > d and regions['l2'] > d and regions['l1'] > d :
    #     state_description = 'case 6 - RTpos'
    #     print ('case 6 - RTpos')
    #     change_state(1)

    # elif regions['r1'] < d and regions['r2'] < d and regions['r3'] < d and regions['r4'] > d and regions['r5'] > d and regions['l5'] > d and regions['l4'] > d and regions['l3'] > d*1.5 and regions['l2'] > d*1.5 and regions['l1'] > d :
    #     state_description = 'case 7 - RTinc'
    #     print ('case 7 - RTinc')
    #     change_state(2)


    # elif regions['r1'] < d and regions['r2'] < d and regions['r3'] < d and regions['r4'] > d and regions['r5'] > d and regions['l5'] > d and regions['l4'] > d and regions['l3'] > d and regions['l2'] > d and regions['l1'] > d*1.5 :
    #     state_description = 'case 8 - RTstp'
    #     print ('case 8 - RTstp')
    #     change_state(2)

    # else:
    #     if command_ == 'l':
    #         uwu = 3
    #     elif command_ == 'r':
    #         uwu = 2
    #     elif command_ == 'f':
    #         uwu = 1
    #     else:
    #         uwu = 0
            
    #     state_description = 'unknown case'
    #     uwu = input('choose ur path: stop (0) forward (1) right (2) left (3)')
    #     change_state(uwu)
    # else:
    #     state_description = 'unknown case'
    #     if uwu == 0:
    #         stop()
    #         print('stop')

    #     elif uwu == 1:
    #         straight()
    #         print('march')

    #     elif uwu == 2:
    #         followleft()
    #         print('left')

    #     elif uwu == 3:
    #         followright()
    #         print('reight')

       # rospy.loginfo(regions)


def path_width():
    global idist_s
    Dref_r = math.sqrt(regions_['r1r']*regions_['r1r'] + regions_['r2r']*regions_['r2r'] - 2*regions_['r1r']*regions_['r2r']*0.93969)
    Aref_r = math.asin(regions_['r2r'] * 0.34202/Dref_r)
    Check_r = Dref_r * 2.87938 * math.sin(Aref_r)    #1.4619 = 1/2/sin(10)
    Diff_r = abs(1 - regions_['rmr'] / Check_r)          #ratio between theoretical reading and actual reading
    Aref_r = 90 - Aref_r/3.1415926*180

    Dref_l = math.sqrt(regions_['l1l']*regions_['l1l'] + regions_['l2l']*regions_['l2l'] - 2*regions_['l1l']*regions_['l2l']*0.93969)
    Aref_l = math.asin(regions_['l2l'] * 0.34202/Dref_l)
    Check_l = Dref_l * 2.87938 * math.sin(Aref_l)    #1.4619 = 1/2/sin(10)
    Diff_l = abs(1 - regions_['lml'] / Check_l)          #ratio between theoretical reading and actual reading
    Aref_l = 90 - Aref_l/3.1415926*180

    #print(Diff_l, "-", Diff_r)
    #print (Aref_l, 'OwO' , Aref_r)
    if Aref_l < 10 and Aref_r < 10 and Diff_l < 0.1 and Diff_r < 0.1:
        width = regions_['l1l'] + regions_['r1r']
        idist_s = width
    else:
        width = idist_s

    return width



def followleft():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    d = idist/5#safety distance

    if regions['left'] > d*2:
        if regions['left'] < d*3 and regions['front'] > idist:
            state_description = 'case 1 -forward'
            change_state(1)
        elif regions['left'] > d*3:
            state_description = 'case 2 -turnleft'
            change_state(3)
        elif regions['front'] < idist and (regions['l3'] < d*3 or regions['l5'] < d*3):
            state_description = 'case 3 -turnright'
            change_state(2)
        else:
            state_description = 'unknown case'
            change_state(1)
            print('standard case')

    elif regions['left'] < d*2:
        if regions['right'] > d*2 and regions['front'] > d:
            state_description = 'case 3 -turnright'
            change_state(2)

    # elif regions_['l1l'] > regions_['r1r']*1.5:
    #     change_state(0)
    #     print('u dont no da wae')
    else:
        state_description = 'stop, cannot pass'
        change_state(0)



def followright():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''
    d = idist/5

    if regions['right'] > d*2:
        if regions['right'] < d*3 and regions['front'] > idist:
            state_description = 'case 1 -forward'
            change_state(1)
        elif regions['right'] > d*3:
            state_description = 'case 2 -turnright'
            change_state(2)
        elif regions['front'] < idist and (regions['r3'] < d*3 or regions['r5'] < d*3):
            state_description = 'case 3 -turnleft'
            change_state(3)
        else:
            state_description = 'unknown case'
            change_state(1)
            print('standard case')

    elif regions['right'] < d*2:
        if regions['left'] > d*2 and regions['front'] > d:
            state_description = 'case 3 -turnleft'
            change_state(3)

    else:
        state_description = 'stop, cannot pass'
        change_state(0)
        
def straight():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''
    d = idist/5
    di = d*2/0.707 #sin(45)


    if regions['front'] > idist:
        if regions['r3'] > di and regions['l3'] > di:
            # if regions['left'] > idist and regions['right'] > idist:
            #     change_state(0)
            #     print('pick your wae')  
            # else:
            change_state(1)
        elif regions['l3'] < di and regions['r3'] > di:
            change_state(2)
        elif regions['l3'] > di and regions['r3'] < di:
      	    change_state(3)
        elif regions['l3'] < di and regions['r3'] < di:
            change_state(0)

    else:
        if regions['l1'] < idist:  
            if regions['front'] < idist:
                print('tunring right')
                followleft()
            else:
                print('right turn finished')

        elif regions['r1'] < idist:
            if regions['front'] < idist:
                print('tunring left')
                followright()
            else:
                print('left turn finished')

        elif regions['l3'] < idist*1.6 and regions['r3'] < idist*1.4:  
            change_state(0)
            print ('pick ur wae')
        elif regions['l3'] < idist*1.4 and regions['r3'] < idist*1.6:  
            change_state(0)
            print ('pick ur wae')



        



def turnleft():
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.3
    return msg

def turnright():
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = 0.3
    return msg

def stop():
    msg = Twist()
    msg.linear.x = 0.0
    return msg

def forward():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)

    rospy.Subscriber('user_input', String, callback)
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = stop()
        elif state_ == 1:
            msg = forward()
        elif state_ == 2:
            msg = turnright()
        elif state_ == 3:
            msg = turnleft()
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()