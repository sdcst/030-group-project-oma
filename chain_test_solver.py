import matplotlib.pyplot as plt
import cv2
import numpy as np

dt=0.00001
class chain_env:
    
    def __init__(self, start_coord, num_joints = 4, lengnth = 1.2 ,mass_per_vert=1.0,
                    distplay_size = 4):

        self.base_coord = start_coord
        self.num_joints = num_joints
        self.mass  = mass_per_vert
        self.distplay_size = distplay_size

        lenth_per_vert = lengnth/(num_joints + 1)

        init_X  = (start_coord[0]+np.linspace(0, lengnth, num_joints)).reshape(num_joints, 1)
        init_Y = np.zeros(init_X.shape).reshape(num_joints, 1)

        self.pos_vert = np.concatenate((init_X, init_Y), axis=1)
        self.vel_vert = np.zeros(self.pos_vert.shape)


    def step(self):
        force_exturnal = np.array([0, - 1.0 * self.mass])
        
        ##print(self.pos_vert)
        
        self.total_force_list = [np.array([0,0])]
        for i in range(1, self.num_joints): # dont update 0
            total_force = force_exturnal
            #print("FOR VERT "+str(i)+":")
            #print("force_exturnal: ", force_exturnal)
            Pi = self.pos_vert[i] - self.pos_vert[i-1]
            Vi = self.vel_vert[i] - self.vel_vert[i-1]
            attraction_speed = np.dot(Vi, Vi)
            dist = np.dot(Pi, Pi)
            ##print("UPPER")
            ##print("upper vert distance:", Pi)
            ##print("upper vert velocity:", Vi)
            ##print("total velocity squared:", attraction_speed)
            ##print("total dist squared:", dist)
            Lambda_1 = -(np.dot(force_exturnal , Pi) +self.mass * attraction_speed)/dist
            ##print("correctave force lambda: ", Lambda_1)
            Fc = Lambda_1 * Pi
            total_force += Fc * 2
            #print("correctave force 1: ", Fc)

            if i+1<self.num_joints: # if its the last jont then dont run below



                Pi = self.pos_vert[i+1] - self.pos_vert[i]
                Vi = self.vel_vert[i+1] - self.vel_vert[i]
                ##print("LOWER")
                ##print("lower vert distance:", Pi)
                ##print("lower vert velocity:", Vi)
                ##print("total velocity squared:", attraction_speed)
                ##print("total dist squared:", dist)


                Lambda_2 = -(np.dot(force_exturnal , Pi) +self.mass * attraction_speed)/dist
                ##print("correctave force lambda: ", Lambda_1)
                Fc = Lambda_2 * Pi
                #print("correctave force 2: ", Fc)
                total_force += Fc * 2
                ##print("total_force: ", total_force)
                ##print("------")
                
                ###print(i, Pi, total_force, Lambda_1, Lambda_2)
            #print("total_force",total_force)
            #print("appended1")
            
            self.total_force_list.append(total_force)
            #print("----")
        
        for i in range(self.num_joints):
            #print("FOR VERT "+str(i)+":")
            #print("total_force",self.total_force_list[i])
            self.vel_vert[i] += self.total_force_list[i]*dt/self.mass
            self.pos_vert[i] += self.vel_vert[i]*dt
        #print("----")
        
    def draw(self):
        canvas = np.zeros([512, 512, 3])
        for i in range(self.num_joints):
            #normal_coord = self.pos_vert[i]/self.distplay_size-self.distplay_size/2
            ###print(normal_coord)
            coord = self.pos_vert[i]/self.distplay_size*512+256
            coord = np.array(coord, dtype=np.int32)
            ##print(coord)
            canvas=cv2.circle(canvas, coord, 4, (250, 10, 10), -1)

            coord = (self.pos_vert[i]+self.vel_vert[i]/10)/self.distplay_size*512+256
            coord = np.array(coord, dtype=np.int32)

            ##print(coord)
            canvas=cv2.circle(canvas, coord, 4, (.1, .9, .1), -1)

            coord = (self.pos_vert[i]+self.total_force_list[i]/10)/self.distplay_size*512+256
            coord = np.array(coord, dtype=np.int32)

            ##print(coord)
            canvas=cv2.circle(canvas, coord, 4, (.1, .1, .9), -1)
        
        return canvas

        
            


test_env = chain_env(np.array([0.1, 0.2]))


for t in range(0, 10000000):
    test_env.step()

    if t%10000==0:
        #test_env.step()
        img = test_env.draw()

        cv2.imshow("test",img)
        cv2.waitKey(1)
        print(test_env.pos_vert)