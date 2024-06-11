import cv2
import numpy as np
import time
from spring_test_solver_v5 import spring

varyable_name_list = ["timestep", "method", "gravity", "floor", "wall", 
"mass", "spring constant", "damping", "framerate", "resolution"] # FUCK DISPLAY SIZE!!!!!

def RUN_SIM(params, positions, connections, locklist):
    print("PARAMS: ",params)
    print("positions: ",positions)
    print("connections: ",connections)
    print("locklist: ",locklist)
    spring_obj = spring(positions, connections, params, locklist)
    SPF = 1/params[-2]
    T0 = time.time()
    i=0
    while True:
        spring_obj.step()
        i+=1
        if abs(time.time()-T0)>=SPF:
            img = spring_obj.draw()
            cv2.imshow("img",img)
            cv2.waitKey(5)


            





    
