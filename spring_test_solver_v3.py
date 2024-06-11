import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
# LETS GOOO!!!!!
dt=0.001

class spring:

    def __init__(self, init_P_list, length_connections_matrix, locklist=None, method="RK4"):
        self.T0=time.time()

        if type(locklist)!=np.ndarray:
            self.locklist = np.zeros(init_P_list.shape[0], dtype=bool)
            self.locklist[0]=True
        else:
            self.locklist = locklist

        self.P_list=init_P_list
        self.V_list = np.zeros(init_P_list.shape)
        self.length_connections_matrix=length_connections_matrix

        self.k_value  = 6
        self.damp_value = -1.0
        self.mass = 2
        self.display_size = 32
        self.method=method
    

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

        self.force_towards_display = np.zeros(self.length_connections_matrix.shape)
        if self.method=="RK4":
            for idx_point, point in enumerate(self.P_list):
                vel = self.V_list[idx_point]

                k1_x = vel#self.calc_x(1, point, vel)
                k1_v = self.calc_v(idx_point, point, vel)

                k2_x = vel+ dt * k1_v/2#self.calc_x(2, point + dt * k1_x/2, vel+ dt * k1_v/2)
                k2_v = self.calc_v(idx_point, point + dt * k1_x/2, vel+ dt * k1_v/2)
            
                k3_x = vel+ dt * k2_v/2#self.calc_x(0, point + dt * k2_x/2, vel+ dt * k2_v/2)
                k3_v = self.calc_v(idx_point, point + dt * k2_x/2, vel+ dt * k2_v/2)

                k4_x = vel+ dt * k3_v#self.calc_x(0, point + dt * k3_x, vel+ dt * k3_v)
                k4_v = self.calc_v(idx_point, point + dt * k3_x, vel+ dt * k3_v)

                self.V_list[idx_point]  += dt * (k1_v + 2*k2_v + 2*k3_v + k4_v)/6
                self.P_list[idx_point] +=  dt * (k1_x + 2*k2_x + 2*k3_x + k4_x)/6
                #self.force_towards_display[idx_point]=(force_towards_display_1+force_towards_display_2+
                #                            force_towards_display_3+force_towards_display_4)
            


        elif self.method=="EULER" or self.method=="MARCUS":
            for idx_point, point in enumerate(self.P_list):
                vel = self.V_list[idx_point]
                
                MAT = self.calc_v(idx_point, self.P_list[idx_point], self.V_list[idx_point])
                self.V_list[idx_point]+=MAT*dt
                #self.force_towards_display[idx_point]=MAT[1]
                self.P_list[idx_point] += vel#self.calc_x(1, self.p1, self.v1)*dt
    

    def calc_v(self, main_idx, x, v):




        
    


        if self.locklist[main_idx]: # if locked dont update position or velocity
            main_accel=0

        else:
            main_force = 0
            for partner_idx, partner_point in enumerate(self.P_list):

                default_length = self.length_connections_matrix[main_idx][partner_idx]

                if default_length!=None: # if there isnt a connection then dont bother


                    dist = np.sqrt(np.sum((partner_point-x)**2))
                    normal_vec = (partner_point-x)/dist
                    vel_towards = np.dot(v,normal_vec) # CORRECT VELOCITY!!!!


                    force_towards = self.k_value * (dist-default_length) + self.damp_value * vel_towards

                    
                    
                    main_force+= normal_vec*force_towards

            main_accel = main_force/self.mass + np.array([0,-12.0])

        return main_accel




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

                    thick = 3#int((abs(self.force_towards_display[main_idx, partner_idx])**0.5+1)/40)
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

if __name__ == "__main__":
    bridge_pos = np.array([[-3.0, 0.00], [-1.0, 1.0], [-1.0, -1.0], [1,1.0], [1.0, -1.1], [3.0, 0.0]])*5
    bridge_pos += np.random.random(bridge_pos.shape)*0.1
    bridge_conn = np.array([[None, 1.0, 1.0, None, None, None],
                            [1.0, None, 5.0, 2.0, 2.0, None],
                            [1.0, 5.0, None, 2.0, 2.0, None],
                            [None, 2.0, 2.0, None, 5.0, 2.0],
                            [None, 2.0, 2.0, 5.0, None, 2.0],
                            [None, None, None, 1.0, 1.0, None]])
    brideg_lock = np.array([True, False, False, False, False, True])

    #test_spring = spring(np.array([[0.0, 0.01], [8.0, 0.1], [1.4, 1.4]]), np.array([[None, 3.0, 4.0], [3.0, None, 4.0], [4.0, 4.0, None]]), locklist=None, method="RK4")
    test_spring = spring(bridge_pos, bridge_conn, brideg_lock, method='RK4')

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