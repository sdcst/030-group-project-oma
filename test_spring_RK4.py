from cv2.gapi.streaming import timestamp
import matplotlib.pyplot as plt
import cv2
import numpy as np
import time
dt=0.01

class spring:

    def __init__(self, p0, p1, method):

        self.method=method

        self.p0 = p0
        self.p1 = p1

        self.v0 = np.array([0,0.0])
        self.v1 = np.array([0,0.0])

        self.k_value  = 25
        self.damp_value = -16
        self.mass_0 = 2.1
        self.length = 5
        self.display_size = 32
    

    def calc_v(self, t, x, v):
        #print("p1???: ",self.p1)

        dist = np.sqrt(np.sum((self.p0-x)**2))
        normal_vec = (self.p0-x)/dist
        vel_towards = np.dot(v,normal_vec)

        force_towards = self.k_value/self.mass_0 * (dist-self.length) + self.damp_value/self.mass_0 * vel_towards

        

        force_vec = normal_vec*force_towards + np.array([0, -19])*self.mass_0

        return force_vec/self.mass_0

    def calc_x(self, t, x, v):

        return v

    def step(self):


        if self.method=="RK4":
        
            k1_x = self.calc_x(1, self.p1, self.v1)
            k1_v = self.calc_v(1, self.p1, self.v1)

            k2_x = self.calc_x(2, self.p1 + dt * k1_x/2, self.v1+ dt * k1_v/2)
            k2_v = self.calc_v(2, self.p1 + dt * k1_x/2, self.v1+ dt * k1_v/2)
        
            k3_x = self.calc_x(0, self.p1 + dt * k2_x/2, self.v1+ dt * k2_v/2)
            k3_v = self.calc_v(0, self.p1 + dt * k2_x/2, self.v1+ dt * k2_v/2)

            k4_x = self.calc_x(0, self.p1 + dt * k3_x, self.v1+ dt * k3_v)
            k4_v = self.calc_v(0, self.p1 + dt * k3_x, self.v1+ dt * k3_v)
            #time.sleep(0.5)

            

            self.v1 += dt * (k1_v + 2*k2_v + 2*k3_v + k4_v)/6
            self.p1 +=  dt * (k1_x + 2*k2_x + 2*k3_x + k4_x)/6
        elif self.method=="EULER":
            self.v1 += self.calc_v(1, self.p1, self.v1)*dt
            self.p1 += self.calc_x(1, self.p1, self.v1)*dt

    def draw(self):

        canvas = np.zeros([512, 512, 3])

        coord = self.p1/self.display_size*512+256
        coord = np.array(coord, dtype=np.int32)
        thick = int(np.sum(self.v1**2)**0.5)

        if thick!=0:
            canvas=cv2.line(canvas, np.array([256, 256]), coord, (0.1, 0.9, 0.1), thick)
        canvas=cv2.circle(canvas, coord, 4, (.1, .1, .9), -1)
        canvas=cv2.circle(canvas, np.array([256,256]), 16, (.9, .1, .9), -1)
        
        return canvas


test_spring_RK4 = spring(np.array([0.0,0]),np.array([-2.0,-5]), "RK4")
test_spring_EUL= spring(np.array([0.0,0]),np.array([-2.0,-5]), "EUL")

for i in range(0,1000000):
    test_spring_RK4.step()
    test_spring_EUL.step()

        


    if i%10==0:
        canvas_RK4 = np.flip(test_spring_RK4.draw(), axis=0)
        canvas_EUL = np.flip(test_spring_EUL.draw(), axis=0)
        #plt.plot(test_spring.p0[0],test_spring.p0[1])
        
        #plt.plot(x,y, 'o')
        #plt.show()
        cv2.imshow("RK4",canvas_RK4)
        cv2.waitKey(5)
        cv2.imshow("EUL",canvas_RK4)
        cv2.waitKey(5)