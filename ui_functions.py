import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
from user_ui import *

main_title_string = "   ____    __  __                            _____   _____   __  __      __      __  ____  ←  / __ \  |  \/  |     /\                   / ____| |_   _| |  \/  |     \ \    / / |___ \ ← | |  | | | \  / |    /  \       ______    | (___     | |   | \  / |      \ \  / /    __) |← | |  | | | |\/| |   / /\ \     |______|    \___ \    | |   | |\/| |       \ \/ /    |__ < ← | |__| | | |  | |  / ____ \                ____) |  _| |_  | |  | |        \  /     ___) |←  \____/  |_|  |_| /_/    \_\              |_____/  |_____| |_|  |_|         \/     |____/ "


def title():

    for char in main_title_string:

        if char == "←":
            print("")
        else:
            print(char, end="")
    print(" ")
    print("\x1B[3mone man army\x1B[0m")
    print("code by Kieran Stuyt and following libaries: pygame, cv2, and numpy")
    print("\n")


def fake_loading(dot_num=3, time_per_dot=0.8):

    for i in range(dot_num-1):
        print(".", end="")
        time.sleep(time_per_dot)
    print(".")
    time.sleep(time_per_dot)


varyable_name_list = ["timestep", "method", "gravity", "floor", "wall", 
"mass", "spring constant", "damping", "framerate", "resolution"]

def intrinsic_params(params):
    EXIT_COMMAND=False

    while EXIT_COMMAND==False:
        print("\n")
        print("\n")
        print("---- INTRINSIC PARAMETERS  -----")
        print("to change a varable, type its name followed by a \":\" charactrue with the new value afterwards")
        print("==========================")
        print(" -- simulation -- ")

        print(" - timestep | (float) "+str(params[0]))
        print(" - method | (NAH/MARCUS/RK4) "+str(params[1]))
        print(" - gravity | (float) "+str(params[2])+" m/s^2")

        print(" - floor | (float, type NA for no floor) "+str(params[3]))
        print(" - walls | (float, walls will be symetric on either side, type NA for no walls) "+str(params[4])+" m/s^2")

        print("\n")

        print(" -- softbody --")
        print(" - mass | (float) "+str(params[5])+" kg per node")
        print(" - spring constant | (float) "+str(params[6])+" N/m")
        print(" - damping | (float) "+str(params[7])+" kg/s")

        print("\n")
        print(" -- display --")

        print(" - framerate | (int) "+str(params[8])+" FPS")
        print(" - resolution | (int) "+str(params[9])+" px")
        print("type the varable name then \":\" charactrue with the new value afterwards ")
        OPTION = input(" or type EXIT in all caps to return to menu: ")
        if OPTION=="EXIT":
            EXIT_COMMAND=True
        else:
            try:
                seporation_idx = OPTION.index(":")
                varable_name = OPTION[:seporation_idx]
                new_value = OPTION[(seporation_idx+1):]
                varable_idx = varyable_name_list.index(varable_name)

                if varable_idx==1:
                    if new_value=="RK4" or new_value=="MARCUS" or new_value=="NAH":
                        params[varable_idx] = new_value
                        print("WORD!!!")
                    else:
                        print("--invalid method!, keep in mind that these are capital sensitave!--")
                elif varable_idx==8 or varable_idx==9:
                    params[varable_idx]=int(new_value)
                else:
                    params[varable_idx]=float(new_value)
            except:
                print("--INVALID INPUT--")

    return params

# DATA ---
bridge_pos = np.array([[-3.0, 0.00], [-1.0, .5], [-1.0, -0.51], [1,0.5], [1.0, -0.5], [3.0, 0.0]])*1.5
bridge_conn = np.array([[None, 1.8, 1.8, None, None, None],
                        [1.8, None, 2.8, 2.8, 2.8, None],
                        [1.8, 2.8, None, 2.8, 2.8, None],
                        [None, 2.8, 2.8, None, 2.8, 1.8],
                        [None, 2.8, 2.8, 2.8, None, 1.8],
                        [None, None, None, 1.8, 1.8, None]])
bridge_lock = np.array([True, False, False, False, False, True])
bridge_ex = [0.01, "RK4", -12.0, None, None,3.4, 100, -0.1, 12, 128]

ball_pos = np.array([[-1.0, 0], [0.0, 0], [1.0, 0],
                        [2.0, 1], [2.0, 2], [2.0, 3], 
                        [1.0, 4], [0.0, 4], [-1, 4],
                        [-2, 3], [-2, 2], [-2, 1],
                        [0,2]])*3

ball_list = []
for i in range(12):
    angle = 2*np.pi*i/12
    ball_list.append(np.array([np.cos(angle), np.sin(angle)+1])*8)

for i in range(8):
    angle = 2*np.pi*i/8
    ball_list.append(np.array([np.cos(angle), np.sin(angle)+4/3])*6)

for i in range(6):
    angle = 2*np.pi*i/6
    ball_list.append(np.array([np.cos(angle), np.sin(angle)+1.8])*4.5)


for i in range(6):
    angle = 2*np.pi*i/6
    ball_list.append(np.array([np.cos(angle), np.sin(angle)+4])*2)


ball_pos = np.array(ball_list)




ball_conn = np.zeros([ball_pos.shape[0], ball_pos.shape[0]])
for i in range(ball_pos.shape[0]):

    for j in range(ball_pos.shape[0]):

        if i==j:
            ball_conn[i,j]=None
        else:
            dist = round(np.sqrt(np.sum((ball_pos[i]-ball_pos[j])**2)),1)
            if dist<5:
                ball_conn[i,j]=dist
            else:
                ball_conn[i,j]=None


ball_lock = np.zeros(ball_pos.shape[0], dtype=bool)

ball_ex = [0.01, "RK4", -12.0, None, None,0.7, 400, -0.01, 12, 128]


x = np.arange(0, 3)
y = np.arange(0, 6)
x, y = np.meshgrid(x, y)
x=x.reshape(3, 6, 1)
y=y.reshape(3,6,1)
jell_pos = np.concatenate((x, y), axis=-1)
jell_pos = np.array(jell_pos.reshape(3*6, 2), dtype=float)*2



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
ball_ex = [0.01, "RK4", -12.0, None, None,0.7, 400, -0.01, 12, 128]
# DATA ---



def create_softbody(params):
    print("--- CREATE SOFTBODY MENU ---")
    
    EXIT=False
    EX_PARAMS=None
    while EXIT==False:
        OPTION = input("would you like to set a default or create a new softbody? (NEW/DEAFAULT): ")
        if OPTION=="NEW":
            POS_LIST, CONN_LIST, LOCK_LIST = user_create_softbody(params)
            EXIT=True
        elif OPTION=="DEAFAULT":
            print("================")
            print("option #1: bridge")
            print("option #2: ball")
            print("option #3: small cube")
            print("option #4: large cube")
            print("option #5: pinned cloth")
            DEAFAULT_OPTION = input("what deafault would you like to use? (1-5):")
            try:
                if 0<int(DEAFAULT_OPTION)<6:
                    EXTRINSIC_PARAM_DEFAULT=input("would you also like to change the extrisic paras to the recommended settings? (Y/N):")
                    if EXTRINSIC_PARAM_DEFAULT=="Y":
                        EX_PARAMS=EX_PARAMS_DAFAULTS[DEAFAULT_OPTION]

                    POS_LIST = POS_DEAFAULT_LIST[DEAFAULT_OPTION]
                    CONN_LIST = CONN_DEAFAULT_LIST[DEAFAULT_OPTION]
                    LOCK_LIST = LOCK_DEAFAULT_LIST[DEAFAULT_OPTION]
                    print("you have selected option:",DEAFAULT_OPTION)
                    fake_loading(time_per_dot=0.4)
                    EXIT=True
                    
                else:
                    print("invalid deafualt value selected!")

            except:
                print("invalid deafualt value selected!")

    return POS_LIST, CONN_LIST, LOCK_LIST, EX_PARAMS

  


if __name__ == "__main__":
    title()
    #fake_loading()
    intrinsic_params(["A","B","B","B","B","B","B","B","B","B","B","B","B"])