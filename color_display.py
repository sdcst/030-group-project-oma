import matplotlib.pyplot as plt
import numpy as np


def HSV2RGB(HSV):
    H, S, V = HSV[0], HSV[1], HSV[2]
    if H<0:
        H = 360 + H
    if H>360:
        H = H%360
    
    S/=100
    V/=100

    C = V * S
    H_prime=H/60
    X = C * (1 - abs(H_prime % 2 -1))
    m = V-C

    if 0<=H<60:
        RGB = np.array([C, X, 0])
    elif 60<=H<120:
        RGB = np.array([X, C, 0])
    elif 120<=H<180:
        RGB = np.array([0, C, X])
    elif 180<=H<240:
        RGB = np.array([0, X, C])
    elif 240<=H<300:
        RGB = np.array([X, 0, C])
    elif 300<=H<=360:
        RGB = np.array([C, 0, X])
    
    try:
        RGB+=1
        RGB-=1
    except:
        print(H)
        exit()
    
    RGB = (RGB+m)*255
    return RGB



Color_0 = np.array((235, 100, 100)) # really blue
Color_1 = np.array((290, 70, 80)) # kinda blue
Color_2 = np.array((330, 70, 80)) # kinda red
Color_3 = np.array((360, 100, 100)) # really red
def force_color(force, scale=1):

    force = force/scale

    if -2>force:
        beta_t = np.exp(-0.02*(force+2)**2)
        HSV = Color_1*beta_t + (1-beta_t)*Color_0
    elif force>2:
        beta_t = np.exp(-0.02*(force-2)**2)
        HSV = Color_2*beta_t + (1-beta_t)*Color_3   
    else:
        beta_t = -0.25*force + 0.5
        HSV = Color_2*(1-beta_t) + Color_1 * beta_t

    RGB = HSV2RGB(HSV)


    return RGB


if __name__ == "__main__":
    print(HSV2RGB(np.array((240, 100, 100)))) # check
    print(HSV2RGB(np.array((24, 60, 90)))) # check
    print(HSV2RGB(np.array((70, 30, 20)))) # check
    print(HSV2RGB(np.array((70, 30, 80)))) # 
    print(HSV2RGB(np.array((70, 90, 10))))


    img = np.zeros([24, 1, 3])

    for i in range(24):
        force = 6*i/24 - 3
        HSV = force_color(force, scale=.4)
        
        #RGB = HSV2RGB(HSV)
        print(HSV)
        img[i] = np.array([HSV])

    img = np.array(img ,dtype=int)

    plt.imshow(img)
    plt.show()
