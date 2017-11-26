#!/usr/bin/env python
import rospy
import sys
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d
import numpy as np
#from scipy import genfromtxt

#option1: http://www.turbare.net/transl/scipy-lecture-notes/intro/matplotlib/matplotlib.html
#option2: http://d.hatena.ne.jp/white_wheels/20100327/p3

if __name__=="__main__":
    rospy.init_node('grasp_planning_plot')

    file_name = rospy.get_param("~file_name")
    object = rospy.get_param("~object")

    fig = pyplot.figure()
    d_list = []
    delta_list = []
    state_list = []
    max_tau_list = []
    max_thrust_list = []
    min_thrust_list = []

    valid_d_list = []
    valid_delta_list = []
    valid_state_list = []
    valid_max_tau_list = []
    valid_max_thrust_list = []
    valid_min_thrust_list = []

    v_valid_lower_bound_phi =  -0.221 #-0.224796
    v_valid_upper_bound_phi =  -0.134 #-0.137796
    v_valid_lower_bound_d = 0.102 #0.105
    v_valid_upper_bound_d = 0.132 #0.135

    if object == "convex" :
        ax = Axes3D(fig)
        #ax.set_xlabel("d [m]")
        #ax.set_ylabel("delta [rad]")
        #ax.set_zlabel("norm of torque")

    for line in open(file_name, 'r'):

        itemList = line.split('\t')

        if object == "convex" :
            d = float(itemList[1])
            delta = float(itemList[2])
            state = float(itemList[3])
            max_tau = float(itemList[4])
            max_thrust = float(itemList[5])
            min_thrust = float(itemList[6])

            if max_tau != -1 :
                if d > v_valid_lower_bound_d and d < v_valid_upper_bound_d and delta > v_valid_lower_bound_phi and delta < v_valid_upper_bound_phi:
                    valid_d_list.append(d)
                    valid_delta_list.append(delta)
                    valid_state_list.append(state)
                    valid_max_tau_list.append(max_tau)
                    valid_max_thrust_list.append(max_thrust)
                    valid_min_thrust_list.append(min_thrust)
                else:
                    d_list.append(d)
                    delta_list.append(delta)
                    state_list.append(state)
                    max_tau_list.append(max_tau)
                    max_thrust_list.append(max_thrust)
                    min_thrust_list.append(min_thrust)

        if object == "cylinder" :
            delta = float(itemList[0])
            state = float(itemList[1])
            max_tau = float(itemList[2])
            max_thrust = float(itemList[3])
            min_thrust = float(itemList[4])

            if max_tau != -1:
                delta_list.append(delta)
                state_list.append(state)
                max_tau_list.append(max_tau)
                max_thrust_list.append(max_thrust)
                min_thrust_list.append(min_thrust)


    if object == "convex" :

        ax.set_xlim(0.0, 0.2)
        ax.set_ylim(-0.3, 0.2)
        pyplot.tick_params(labelsize=18)

        #cm = pyplot.get_cmap("winter")
        #col = [cm(float(i)/(len(max_tau_list))) for i in xrange(0, len(max_tau_list))]

        '''
        # max_tau
        ax.scatter(d_list, delta_list, max_tau_list, s= 1, alpha=0.5)
        ax.scatter(valid_d_list, valid_delta_list, valid_max_tau_list, s= 1, color="red")
        ax.set_zlim(2.0, 4.0)
        max_value =3.09678
        '''

        '''
        # min_thrust
        ax.scatter(d_list, delta_list, min_thrust_list, s= 1, alpha=0.5)
        ax.scatter(valid_d_list, valid_delta_list, valid_min_thrust_list, s= 1, color="red")
        ax.set_zlim(9.7, 10.0)
        max_value =9.883
        '''

        '''
        # max_thrust
        ax.scatter(d_list, delta_list, max_thrust_list, s= 1, alpha=0.5)
        ax.scatter(valid_d_list, valid_delta_list, valid_max_thrust_list, s= 1, color="red")
        ax.set_zlim(11.3, 11.55)
        max_value =11.3885
        '''

        # state
        ax.scatter(d_list, delta_list, state_list, s= 1, alpha=0.5)
        ax.scatter(valid_d_list, valid_delta_list, valid_state_list, s= 1, color="red")
        ax.set_zlim(12, 12.6)
        max_value = 12.2

        start1 = []
        start2 = []
        start3 = []
        start1.append(0.12)
        start2.append(-0.181296)
        start3.append(max_value)

        ax.scatter(start1, start2, start3, s= 150, c='black')
        #ax.scatter(start1, start2, start3, s= 100, c='violet')
        pyplot.show()

    if object == "cylinder" :
        #fig, ax = pyplot.subplots()
        #pyplot.xlabel("xlabel", fontsize=18)
        #pyplot.ylabel("ylabel", fontsize=18)
        pyplot.rcParams["font.size"] = 18
        pyplot.axvspan(-0.0730052, 0.0139948, alpha=0.5, color='red')

        '''
        # max tau
        y_min = min(max_tau_list) - 0.005
        y_max = max(max_tau_list) + 0.005
        pyplot.ylim((y_min,y_max))
        pyplot.plot(delta_list, max_tau_list, color='black')
        '''

        '''
        # min_thrust
        y_min = min(min_thrust_list) - 0.005
        y_max = max(min_thrust_list) + 0.005
        pyplot.ylim((y_min,y_max))
        pyplot.plot(delta_list, min_thrust_list, color='black')
        '''

        '''
        # max_thrust
        y_min = min(max_thrust_list) - 0.005
        y_max = max(max_thrust_list) + 0.005
        pyplot.ylim((y_min,y_max))
        pyplot.plot(delta_list, max_thrust_list, color='black')
        '''

        # state
        y_min = min(state_list) - 0.005
        y_max = max(state_list) + 0.005
        pyplot.ylim((y_min,y_max))
        pyplot.plot(delta_list, state_list, color='black')

        pyplot.vlines(x=-0.0295052, ymin=y_min, ymax=y_max, color='blue',  linestyles ='dashed', zorder=2)
        pyplot.show()

