import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
from ui_functions import *
from ui_brute_sim import *

x = np.arange(0, 2)
y = np.arange(0, 7)
x, y = np.meshgrid(x, y)
x=x.reshape(2, 7, 1)
y=y.reshape(2,7,1)
jell_pos = np.concatenate((x, y), axis=-1)
jell_pos = np.array(jell_pos.reshape(2*7, 2), dtype=float)*2



jell_conn = np.zeros([jell_pos.shape[0], jell_pos.shape[0]])
for i in range(jell_pos.shape[0]):

    for j in range(jell_pos.shape[0]):

        if i==j:
            jell_conn[i,j]=None
        else:
            dist = round(np.sqrt(np.sum((jell_pos[i]-jell_pos[j])**2)),1)
            if dist<3:
                jell_conn[i,j]=dist
            else:
                jell_conn[i,j]=None            

jell_lock = np.zeros(jell_pos.shape[0], dtype=bool)
ball_ex = [0.01, "RK4", -12.0, -6.0, 16.0,0.7, 400, -0.01, 12, 128]

varyable_name_list = ["timestep", "method", "gravity", "floor", "wall", "mass","spring constant","damping","framerate", "resolution"]
params = [0.01, "RK4", -9.81, -6.0, 16.0, 0.7, 400.0, -0.01, 12, 128]

if __name__ == "__main__":
    EXIT=False

    while EXIT==False:
        print("\n")
        print("\n")
        print("\n")
        title()
        #fake_loading()
        print("hello and welcome to OMA-SIM!, Kierans softbody simulator,  what would you like to do?")
        print("1 : modify extrinsic parameters")
        print("2 : see advanced settings")
        print("3 : edit softbody shape/connections")
        print("4 : run simulation")
        print("5 : see help page")
        OPTION = input("Enter an interger option between 1 and 5 :")
        print(" ---- ENTERD OPTION #"+OPTION+" ------")
        print("\n")

        
        if OPTION=="1":
            params = intrinsic_params(params)
        elif OPTION=="2":
            print("ERROR: UNFINISHED!")
        elif OPTION=="3":
            positions, connections, locklist, new_params = create_softbody(params)

            if new_params!=None:
                params = new_params

        elif OPTION=="4":
            if params==None:
                print("create positions and connections first!, you dont have those selected!")
            else:

                try:
                    RUN_SIM(params, positions, connections, locklist)
                except:
                    RUN_SIM(ball_ex, jell_pos, jell_conn, jell_lock)
                
        elif OPTION=="5":
            print("ERROR: UNFINISHED!")
        elif OPTION=="w":
            print(ball_ex, jell_pos, jell_conn, jell_lock)
            #exit()
            RUN_SIM(ball_ex, jell_pos, jell_conn, jell_lock)
        else:
            print("--enter a valid option!!!--")
