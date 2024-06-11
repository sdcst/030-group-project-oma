from cv2.gapi.streaming import timestamp
import matplotlib.pyplot as plt
import cv2
import numpy as np

dt=0.0001

class spring:

    def __init__(self, p0, p1):

        self.p0 = p0
        self.p1 = p1

        self.v0 = np.array([0,0.0])
        self.v1 = np.array([0,0.0])

        self.k_value  = 15
        self.damp_value = 10
        self.mass_0 = 2.1
        self.length = 5
        self.display_size = 32
    

    def step(self):
        #print("p1???: ",self.p1)

        dist = np.sqrt(np.sum((self.p0-self.p1)**2))
        vel_towards = np.sqrt(np.sum((self.v0-self.v1)**2))

        self.force_towards = self.k_value/self.mass_0 * (dist-self.length) + self.damp_value/self.mass_0 * vel_towards

        normal_vec = (self.p0-self.p1)/dist

        force_vec = normal_vec*self.force_towards + np.array([0, -19])*self.mass_0

        self.v1 += force_vec*dt/self.mass_0
        self.p1 += self.v1*dt

    def draw(self):

        canvas = np.zeros([512, 512, 3])

        coord = self.p1/self.display_size*512+256
        coord = np.array(coord, dtype=np.int32)
        thick = int(self.force_towards**0.5)

        if thick!=0:
            canvas=cv2.line(canvas, np.array([256, 256]), coord, (0.1, 0.9, 0.1), thick)
        canvas=cv2.circle(canvas, coord, 4, (.1, .1, .9), -1)
        canvas=cv2.circle(canvas, np.array([256,256]), 16, (.9, .1, .9), -1)
        
        return canvas


test_spring = spring(np.array([0.0,0]),np.array([-2.0,-5]))

x=[]
y=[]
xv=[]
yv=[]
for i in range(0,1000000):
    test_spring.step()
    if i%100==0:
        
        print(test_spring.p1)
        x.append(test_spring.p1[0])
        y.append(test_spring.p1[1])

        xv.append(test_spring.v1[0]+test_spring.p1[0])
        yv.append(test_spring.v1[1]+test_spring.p1[1])

    if i%100==0:
        canvas = np.flip(test_spring.draw(), axis=0)
        #plt.plot(test_spring.p0[0],test_spring.p0[1])
        
        #plt.plot(x,y, 'o')
        #plt.show()
        cv2.imshow("img",canvas)
        cv2.waitKey(5)