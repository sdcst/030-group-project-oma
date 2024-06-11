import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
from color_display import *
# LETS GOOO!!!!!
dt=0.01


class spring:

    def __init__(self, init_P_list, length_connections_matrix, locklist=None, method="RK4", ground_value=None, gravity=-12.0):
        self.T0=time.time()
        self.num_points=init_P_list.shape[0]


        if type(locklist)!=np.ndarray:
            self.locklist = np.zeros(init_P_list.shape[0], dtype=bool)
            self.locklist[0]=True
        else:
            self.locklist = locklist

        self.P_list=init_P_list
        self.V_list = np.zeros(init_P_list.shape)
        self.length_connections_matrix=length_connections_matrix


        self.k_value  = 400
        self.damp_value = -.01
        self.mass = 0.7
        self.display_size = 32
        self.method=method
        self.gravity=gravity
        if ground_value==None:
            self.ground_value = ground_value
            self.ground_bool = False
        else:
            self.ground_value = ground_value
            self.ground_bool = True
        self.wall_bool=True
        self.wall_value = 16

        
        self.init_potential_energy = np.sum((self.ground_value- self.P_list[:, 1])*gravity*self.mass)


    

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
                k1_v = self.calculate_velocity(idx_point, point, vel)

                k2_x = vel+ dt * k1_v/2#self.calc_x(2, point + dt * k1_x/2, vel+ dt * k1_v/2)
                k2_v = self.calculate_velocity(idx_point, point + dt * k1_x/2, vel+ dt * k1_v/2)
            
                k3_x = vel+ dt * k2_v/2#self.calc_x(0, point + dt * k2_x/2, vel+ dt * k2_v/2)
                k3_v = self.calculate_velocity(idx_point, point + dt * k2_x/2, vel+ dt * k2_v/2)

                k4_x = vel+ dt * k3_v#self.calc_x(0, point + dt * k3_x, vel+ dt * k3_v)
                k4_v = self.calculate_velocity(idx_point, point + dt * k3_x, vel+ dt * k3_v)

                self.V_list[idx_point]  += dt * (k1_v + 2*k2_v + 2*k3_v + k4_v)/6
                self.P_list[idx_point] +=  dt * (k1_x + 2*k2_x + 2*k3_x + k4_x)/6
                if self.ground_bool:

                    if self.P_list[idx_point][1]<self.ground_value:
                        self.P_list[idx_point][1]=self.ground_value
                        self.V_list[idx_point][1] = -self.V_list[idx_point][1]*0.1

                if self.wall_bool:

                    if self.P_list[idx_point][0]>self.wall_value:
                        self.P_list[idx_point][0]=self.wall_value
                        self.V_list[idx_point][0] = -self.V_list[idx_point][0]*0.1

                    if self.P_list[idx_point][0]<-self.wall_value:
                        self.P_list[idx_point][0]=-self.wall_value
                        self.V_list[idx_point][0] = -self.V_list[idx_point][0]*0.1
                #self.force_towards_display[idx_point]=(force_towards_display_1+force_towards_display_2+
                #                            force_towards_display_3+force_towards_display_4)
            


        elif self.method=="EULER" or self.method=="MARCUS":
            for idx_point, point in enumerate(self.P_list):
                vel = self.V_list[idx_point]
                
                self.V_list[idx_point]+=self.calculate_velocity(idx_point, self.P_list[idx_point], self.V_list[idx_point])*dt/100
                #self.force_towards_display[idx_point]=MAT[1]
                self.P_list[idx_point] += vel#self.calc_x(1, self.p1, self.v1)*dt
                if self.ground_bool:

                    if self.P_list[idx_point][1]<self.ground_value:
                        self.P_list[idx_point][1]=self.ground_value
                        self.V_list[idx_point][1] = -self.V_list[idx_point][1]*0.1
        
        elif self.method =="NAH":
            pass
    

    def calculate_velocity(self, main_idx, x, v):
        if main_idx==3:
            pass
            #exit()

        if self.locklist[main_idx]: # if locked dont update position or velocity
            main_accel=0

        else:
            main_force = 0
            for partner_idx, partner_point in enumerate(self.P_list):

                default_length = self.length_connections_matrix[main_idx][partner_idx]
                #print("----")
                #print("main idx: ",main_idx)
                #print("parter idx: ",partner_idx)
                #print("default_length: ",default_length)
                #print("type ",type(default_length))

                if default_length!=None and not np.isnan(default_length): # if there isnt a connection then dont bother

                        
                    dist = np.sqrt(np.sum((partner_point-x)**2))

                    #print("distance ",dist)

                    normal_vec = (partner_point-x)/dist
                    vel_towards = np.dot(v,normal_vec) # CORRECT VELOCITY!!!!


                    force_towards = self.k_value * (dist-default_length) + self.damp_value * vel_towards

                    
                    
                    main_force+= normal_vec*force_towards

            main_accel = main_force/self.mass + np.array([0,self.gravity])
        
        return main_accel
    

    def find_force(self, main_idx, partner_idx):

        partner_point = self.P_list[partner_idx]
        main_point = self.P_list[main_idx]

        v = self.V_list[main_idx]

        default_length = self.length_connections_matrix[main_idx][partner_idx]
        dist = np.sqrt(np.sum((partner_point-main_point)**2))

        normal_vec = (partner_point-main_point)/dist
        vel_towards = np.dot(v,normal_vec) # CORRECT VELOCITY!!!!


        force_towards = self.k_value * (dist-default_length) + self.damp_value * vel_towards
        potential_energy = self.k_value * (dist-default_length)**2
        return force_towards, potential_energy




    def draw(self):

        canvas = np.zeros([512, 512, 3])
        total_potential_spring=0
        for main_idx, main_point in enumerate(self.P_list):
            main_coord = self.P_list[main_idx]/self.display_size*512+256
            main_coord = np.array(main_coord, dtype=np.int32)
            

            for partner_idx, partner_point in enumerate(self.P_list):

                partner_coord = self.P_list[partner_idx]/self.display_size*512+256
                partner_coord = np.array(partner_coord, dtype=np.int32)
                #print("LINE: ",main_idx,partner_idx,"BOOL",self.length_connections_matrix[main_idx][partner_idx])

                if self.length_connections_matrix[main_idx][partner_idx]!=None and not np.isnan(self.length_connections_matrix[main_idx][partner_idx]):

                    VAL_FORCE, PE=self.find_force(main_idx, partner_idx)
                    if main_idx>partner_idx: # prevents repeats of the same spring stwice
                        total_potential_spring+=0.5*PE/self.num_points

                    #print(self.force_towards[main_idx, partner_idx])

                    thick = 3#int((abs(self.force_towards_display[main_idx, partner_idx])**0.5+1)/40)
                    
                    LINE_COL = force_color(VAL_FORCE, scale=10)
                    LINE_COL = np.array(LINE_COL)/255
                    
                    
                    if thick>0:

                        canvas=cv2.line(canvas, partner_coord, main_coord, LINE_COL, thick)
        kinetic_energy_node=0
        potential_energy_node=0
        #total_momentium=0
        for main_idx, main_point in enumerate(self.P_list):
            main_coord = self.P_list[main_idx]/self.display_size*512+256
            main_coord = np.array(main_coord, dtype=np.int32)
            canvas=cv2.circle(canvas, main_coord, 4, (.1, .1, .9), -1)

            #total_momentium+=np.sum(2*self.V_list[main_idx]**2)**0.5*self.mass/self.num_points
            POT=self.mass*self.gravity*(self.ground_value-self.P_list[main_idx, 1])
            KIN=0.5*np.sum(self.V_list[main_idx]**2)*self.mass/self.num_points
            
            kinetic_energy_node+=0.5*np.sum(self.V_list[main_idx]**2)*self.mass/self.num_points

            potential_energy_node+=self.mass*self.gravity*(self.ground_value-self.P_list[main_idx, 1])/self.num_points
            #canvas=cv2.circle(canvas, np.array([256,256]), 16, (.9, .1, .9), -1)

        TOTAL_NODE_ENERGY=kinetic_energy_node+potential_energy_node
        GROUND_DISPLAY_COORDS=int(self.ground_value/self.display_size*512+256)

        
        
        canvas[:GROUND_DISPLAY_COORDS] = np.array([0.4,0.4,0.4])
        canvas[:, 0:30] = np.array((.9, .9, .9))
        canvas[:int(512*0.9*(total_potential_spring)/(self.init_potential_energy/self.num_points)), 10:15] = np.array((.1, .5, .9))
        canvas[:int(512*0.9*(TOTAL_NODE_ENERGY)/(self.init_potential_energy/self.num_points)), 0:5] = np.array((.9, .5, .0))
        canvas[:int(512*0.9*(TOTAL_NODE_ENERGY+total_potential_spring)/(self.init_potential_energy/self.num_points)), 20:25] = np.array((.3, .9, .3))

        #print("KINETIC ENERGY IN NODES: ",kinetic_energy_node)
        #print("POTENTIAL ENERGY IN NODES: ",potential_energy_node)
        #print("POTENTIAL ENERGY IN SPRINGS: ",total_potential_spring)
        return canvas



if __name__ == "__main__":

    bridge_pos = np.array([[-3.0, 0.00], [-1.0, .5], [-1.0, -0.51], [1,0.5], [1.0, -0.5], [3.0, 0.0]])*1.5

    bridge_conn = np.array([[None, 1.8, 1.8, None, None, None],
                            [1.8, None, 2.8, 2.8, 2.8, None],
                            [1.8, 2.8, None, 2.8, 2.8, None],
                            [None, 2.8, 2.8, None, 2.8, 1.8],
                            [None, 2.8, 2.8, 2.8, None, 1.8],
                            [None, None, None, 1.8, 1.8, None]])

    brideg_lock = np.array([True, False, False, False, False, True])




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
                if dist<10:
                    ball_conn[i,j]=dist
                else:
                    ball_conn[i,j]=None
    print(ball_conn)

    ball_lock = np.zeros(ball_pos.shape[0], dtype=bool)

    '''
    #ball_pos+=(np.random.random(ball_pos.shape)*2-1)/3
    #ball_pos*=np.array([1.2, 1])

    '''
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
    '''

    '''
    '''
    pent_list = []
    for i in range(5):
        angle = 2*np.pi*i/5
        pent_list.append(np.array([np.cos(angle), np.sin(angle)])*2)

    for i in range(5):
        angle = 2*np.pi*i/5+np.pi/5
        pent_list.append(np.array([np.cos(angle), np.sin(angle)])*4)

    pent_pos = np.array(pent_list)
    #pent_pos = np.append(pent_pos, pent_pos+9, axis=0)
   # print(pent_pos.shape)


    pent_conn = np.zeros([pent_pos.shape[0], pent_pos.shape[0]])
    for i in range(pent_pos.shape[0]):

        for j in range(pent_pos.shape[0]):

            if i==j:
                pent_conn[i,j]=None
            else:
                dist = round(np.sqrt(np.sum((pent_pos[i]-pent_pos[j])**2)),1)
                if dist<5:
                    pent_conn[i,j]=dist
                else:
                    pent_conn[i,j]=None

    pent_lock = np.zeros(pent_pos.shape[0], dtype=bool)            

    '''
    print(jell_lock.shape)

    test_spring = spring(jell_pos, jell_conn,locklist=jell_lock, method='RK4', ground_value=-12)

    T0=time.time()
    while True:
        test_spring.step()
            
        
        if (time.time()-T0)>0.01:
            T0=time.time()
            canvas = np.flip(test_spring.draw(), axis=0)
            #plt.plot(test_spring.p0[0],test_spring.p0[1])
            
            #plt.plot(x,y, 'o')
            #plt.show()
            cv2.imshow("img",canvas)
            cv2.waitKey(5)