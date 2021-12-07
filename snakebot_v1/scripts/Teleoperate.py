#!/usr/bin/env python
import rospy

import math
import numpy as np

from std_msgs.msg import Float64

import sys, select, termios, tty


def publisher_nodes(N):
    publishers = []
    for i in range(N):
        pub_name = '/snakebot_v1/joint_{}_position_controller/command'.format(i+1)
        node = rospy.Publisher(pub_name, Float64, queue_size=1000)
        publishers.append(node)
    return publishers

rad = math.radians
def sinewave_params(mode, alpha):


    if mode == 'LinearProgression':
        amplitudes = [np.pi/3* alpha, 0 * abs(alpha)] 
        temp_freqs = [rad(150) , rad(150)]
        spatial_freqs = [2* np.pi/3, 0]
        phase =[0, 0]

    elif mode == 'CaterPillar':
        amplitudes = [np.pi/6* alpha, 0* abs(alpha)]
        temp_freqs = [5* np.pi / 6, 0,  np.pi / 6]
        spatial_freqs = [2* np.pi/3, 0]
        phase =[0, 0]


    elif mode == 'SideWinding':
        amplitudes = [np.pi/6* alpha, np.pi/6* abs(alpha)]
        temp_freqs = [5* np.pi / 6, 5* np.pi / 6]
        spatial_freqs = [2* np.pi/3, 2* np.pi/3]
        phase =[0, 0]

    elif mode == 'Rolling':
        amplitudes = [np.pi/3* alpha, np.pi/3* abs(alpha)]
        temp_freqs = [rad(150) , rad(150)]
        spatial_freqs = [np.pi/2, np.pi/2]
        phase = [0, np.pi/6]

    elif mode == 'LateralUndulation':
        amplitudes = [0* alpha, rad(60)* abs(alpha)] 
        temp_freqs = [rad(150) , rad(150)]
        spatial_freqs = [0, rad(120)]
        phase = [rad(0),rad(0)]

    elif  mode == 'Rotate':
        amplitudes = [rad(90)* alpha, rad(90)* abs(alpha)]
        temp_freqs = [rad(180) , rad(180)]
        spatial_freqs = [rad(90), 0]
        phase = [rad(0),rad(0)]

    return amplitudes, temp_freqs, spatial_freqs, phase


def publish_angles(t, N, mode, alpha, publishers):        

    params = sinewave_params(mode, alpha)
    amplitudes, temp_freqs, spatial_freqs, phases = params
    for i in range(N):
        n = i+1
        a,w,d,phi = amplitudes[n%2], temp_freqs[n%2], spatial_freqs[n%2], phases[n%2]
        link_num = i // 2  

        if mode == 'Rotate':
            angle = (a * math.sin(w*t + link_num*d + phi))*math.pow(1, link_num)            
        elif mode == 'CaterPillar':
            w_o =temp_freqs[2]
            angle = min(a * math.sin(w*t + d + w_o + phi), 0)
        else :
            angle = (a * math.sin(w*t + n*d + phi))*math.pow(-1, link_num)

        #rospy.loginfo('Angle = {} for joint = {}'.format(angle, i+1))
        publishers[i].publish(angle)


msg = """
Control Your Conda!
---------------------------
Moving around:
        w    
   a    s    d
   z    x    c
z/c : roll
w : forward- Linear
s: slither
a/d: rotate in place
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
        
        'w':('LinearProgression', -0.9),
        's':('LateralUndulation', 1.0),
        # 'w':('LateralUndulation', 2.0),        
        'a':('Rotate', -0.7),
        'd':('Rotate', 0.7),
        'q':('SideWinding', 0.9),
        'e':('SideWinding', -0.9),
        'z':('Rolling', 0.6),
        'c':('Rolling', -0.6),
           }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('conda_teleop')

    publishers = publisher_nodes(N=10)

    mode = 'LateralUndulation'
    alpha = 0.6
    status = 0
    count = 0
    rate = rospy.Rate(10) 
    time_begin = rospy.Time.now()
    try:
        print(msg)

        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                mode = moveBindings[key][0]
                alpha = moveBindings[key][1]
                count = 0

                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                print(mode)

            elif key == ' ' or key == 'x' :
                alpha = 0.1
                mode = 'LateralUndulation'
            else:
                count = count + 1
                if count > 4:
                    alpha = 0.1
                    mode = 'LateralUndulation'
                if (key == '\x03'):
                    break

            time = (rospy.Time.now() - time_begin).to_sec()

            # publish mmode and alpha
            publish_angles(time, 10, mode, alpha,  publishers)

            rate.sleep()


    except Exception as e:
        print(e)

    finally:
        publishers[0].publish(0.001)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
