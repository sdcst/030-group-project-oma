import time
import matplotlib.pyplot as plt
import cv2
import numpy as np

dt=0.001

class spring:

    def __init__(self, init_P_list, length_connections_matrix, locklist=None):
        self.T0=time.time()

        if locklist==None:
            self.locklist = np.zeros(init_P_list.shape[0], dtype=bool)
            self.locklist[0]=True
        else:
            self.locklist = locklist

        self.P_list=init_P_list
        self.V_list = np.zeros(init_P_list.shape)
        self.length_connections_matrix=length_connections_matrix

        self.k_value  = 150
        self.damp_value = 10.0
        self.mass = 2.1
        self.display_size = 32
    

    def check_connections(self):
        for main_idx, main_point in enumerate(self.P_list):
            #time.sleep(1)
            print("----")

            for partner_idx, partner_point in enumerate(self.P_list):

                if self.length_connections_matrix[main_idx][partner_idx]!=None:
                    print(str(main_idx)+" is connected to "+str(partner_idx))
                else:
                    print(str(main_idx)+" is NOT connected to "+str(partner_idx))



    def step(self):
        temp_force_list=[]
        for main_idx, main_point in enumerate(self.P_list):
            #print(main_point, main_idx)
            PINT=False

            main_force = 0
            self.force_towards=np.zeros(self.length_connections_matrix.shape)

            if not self.locklist[main_idx]: # if locked dont update position or velocity
                

                
                main_velocity = self.V_list[main_idx]


                for partner_idx, partner_point in enumerate(self.P_list):

                    partner_velocity = self.V_list[partner_idx]
                    default_length = self.length_connections_matrix[main_idx][partner_idx]
                    if PINT:
                        print("DEFAULT LENGTH BETWEN "+str(main_idx)+" and its partner "+str(partner_idx)+": "+str(default_length))

                    if default_length!=None: # if there isnt a connection then dont bother


                        dist = np.sqrt(np.sum((partner_point-main_point)**2))
                        if PINT:
                            print("dist: ",dist)
                        vel_towards = np.sqrt(np.sum((partner_velocity-main_velocity)**2))

                        self.force_towards[main_idx, partner_idx] = self.k_value/self.mass * (dist-default_length) + self.damp_value/self.mass * vel_towards
                        if PINT:
                            print("force_towards: ",self.force_towards[main_idx, partner_idx])
                        
                        normal_vec = (partner_point-main_point)/dist
                        if PINT:
                            print("normal vectoer: ",normal_vec)

                            print("full normal vector: ",normal_vec*self.force_towards[main_idx, partner_idx])

                        main_force+= normal_vec*self.force_towards[main_idx, partner_idx] + np.array([0, -4])*self.mass
                        if PINT:
                            print("main_force: ",main_force)

                
            temp_force_list.append(main_force) # append temp force list regardless if main point is locked, it should be syncted up.

        for idx, point in enumerate(self.P_list):
            if PINT:
                print("point ",idx)
                print("accel ",temp_force_list[idx]*dt/self.mass)
                print("vel ",self.V_list[idx])
                print("pos ",self.P_list[idx])

            self.V_list[idx]  += temp_force_list[idx]*dt/self.mass # EURLER METHOD BAD!!!
            self.P_list[idx] += self.V_list[idx] *dt
        
        if PINT:
            exit()

    def draw(self):

        canvas = np.zeros([512, 512, 3])

        for main_idx, main_point in enumerate(self.P_list):
            main_coord = self.P_list[main_idx]/self.display_size*512+256
            main_coord = np.array(main_coord, dtype=np.int32)
            

            for partner_idx, partner_point in enumerate(self.P_list):

                partner_coord = self.P_list[partner_idx]/self.display_size*512+256
                partner_coord = np.array(partner_coord, dtype=np.int32)
                #print("LINE: ",main_idx,partner_idx,"BOOL",self.length_connections_matrix[main_idx][partner_idx])

                if self.length_connections_matrix[main_idx][partner_idx]!=None:
                    #print(self.force_towards[main_idx, partner_idx])

                    thick = int(abs(self.force_towards[main_idx, partner_idx])**0.5+1)
                    if thick>0:

                        canvas=cv2.line(canvas, partner_coord, main_coord, (0.1, 0.9, 0.1), thick)
        total_kinetic_energy=0
        total_momentium=0
        for main_idx, main_point in enumerate(self.P_list):
            main_coord = self.P_list[main_idx]/self.display_size*512+256
            main_coord = np.array(main_coord, dtype=np.int32)
            canvas=cv2.circle(canvas, main_coord, 4, (.1, .1, .9), -1)

            total_momentium+=np.sum(self.V_list[main_idx]**2)**0.5*self.mass
            total_kinetic_energy+=0.5*np.sum(self.V_list[main_idx]**2)*self.mass
            #canvas=cv2.circle(canvas, np.array([256,256]), 16, (.9, .1, .9), -1)
        canvas[:int(total_kinetic_energy), 10:15] = np.array((.1, .5, .9))
        canvas[:int(total_momentium), 0:5] = np.array((.9, .5, .0))
        print("KINETIC ENERGY IN SYSTEM: ",total_kinetic_energy)
        return canvas


test_spring = spring(np.array([[0.0, 0.01], [3.0, 0.1], [1.4, 1.4]]), np.array([[None, 3.0, 4.0], [3.0, None, 2.0], [4.0, 2.0, None]]))
test_spring.check_connections()

for i in range(0,1000000):
    test_spring.step()
        

    if i%100==0:
        canvas = np.flip(test_spring.draw(), axis=0)
        #plt.plot(test_spring.p0[0],test_spring.p0[1])
        
        #plt.plot(x,y, 'o')
        #plt.show()
        cv2.imshow("img",canvas)
        cv2.waitKey(5)