import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
from color_display import *
# LETS GOOO!!!!!


varyable_name_list = ["timestep", "method", "gravity", "floor", "wall", 
"mass", "spring constant", "damping", "framerate", "resolution"] # FUCK DISPLAY SIZE!!!!!
class spring:

    def __init__(self, init_P_list, length_connections_matrix,params, locklist=None):
        self.dt = params[0]
        self.T0=time.time()
        self.num_points=init_P_list.shape[0]


        if type(locklist)!=np.ndarray:
            self.locklist = np.zeros(init_P_list.shape[0], dtype=bool)

        else:
            if locklist.shape[0]!=0:
                self.locklist = locklist
            else:
                self.locklist = np.zeros(init_P_list.shape[0], dtype=bool)
        #print(params, init_P_list, length_connections_matrix, self.locklist)
        #exit()

        self.P_list=init_P_list
        self.V_list = np.zeros(init_P_list.shape)
        self.length_connections_matrix=length_connections_matrix


        self.k_value  = params[6]
        self.damp_value = params[7]
        self.mass = params[5]
        self.display_size = 32
        self.method=params[1]
        self.gravity=params[2]
        if params[3]==None:
            self.ground_bool = False
        else:
            self.ground_value = params[3]
            self.ground_bool = True

        if params[4]==None:
            self.wall_bool = False
            self.init_potential_energy = 999
        else:
            self.init_potential_energy = np.sum((self.ground_value- self.P_list[:, 1])*params[2]*self.mass)
            self.wall_value = params[4]
            self.wall_bool = True
        print(self.k_value, "K VAL")
        print(self.damp_value, "DAMP VALUE")
        print(self.mass, "MASS")
        print(self.method, "METHOD")
        print(self.gravity,"gravity")
        print(self.ground_bool)
        print(self.wall_bool)

        print(self.P_list, "P LIST!!!!")
        


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
                #print("main point: ",point)
                #print("idx main point",idx_point)
                #print("----")


                vel = self.V_list[idx_point]

                k1_x = vel#self.calc_x(1, point, vel)
                k1_v = self.calculate_velocity(idx_point, point, vel)

                k2_x = vel+ self.dt * k1_v/2#self.calc_x(2, point + self.dt * k1_x/2, vel+ self.dt * k1_v/2)
                k2_v = self.calculate_velocity(idx_point, point + self.dt * k1_x/2, vel+ self.dt * k1_v/2)
            
                k3_x = vel+ self.dt * k2_v/2#self.calc_x(0, point + self.dt * k2_x/2, vel+ self.dt * k2_v/2)
                k3_v = self.calculate_velocity(idx_point, point + self.dt * k2_x/2, vel+ self.dt * k2_v/2)

                k4_x = vel+ self.dt * k3_v#self.calc_x(0, point + self.dt * k3_x, vel+ self.dt * k3_v)
                k4_v = self.calculate_velocity(idx_point, point + self.dt * k3_x, vel+ self.dt * k3_v)

                self.V_list[idx_point]  += self.dt * (k1_v + 2*k2_v + 2*k3_v + k4_v)/6
                self.P_list[idx_point] +=  self.dt * (k1_x + 2*k2_x + 2*k3_x + k4_x)/6
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

            

        elif self.method=="EULER" or self.method=="MARCUS":
            for idx_point, point in enumerate(self.P_list):
                vel = self.V_list[idx_point]
                
                self.V_list[idx_point]+=self.calculate_velocity(idx_point, self.P_list[idx_point], self.V_list[idx_point])*self.dt/100
                #self.force_towards_display[idx_point]=MAT[1]
                self.P_list[idx_point] += vel#self.calc_x(1, self.p1, self.v1)*dt
                if self.ground_bool:

                    if self.P_list[idx_point][1]<self.ground_value:
                        self.P_list[idx_point][1]=self.ground_value
                        self.V_list[idx_point][1] = -self.V_list[idx_point][1]*0.1
        
        elif self.method =="NAH":
            pass
    

    def calculate_velocity(self, main_idx, x, v):


        
        if main_idx in list(self.locklist): # if locked dont update position or velocity
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

                if default_length!=0.0 and default_length!=None and not np.isnan(default_length): # if there isnt a connection then dont bother

                        
                    dist = np.sqrt(np.sum((partner_point-x)**2))
                    #print("---- comparision: ",dist, partner_idx, "------")
                    #print("default_length",default_length)
                    #print("DISTANCE BETWEEN "+str(main_idx)+" AND "+str(partner_idx)+" IS "+str(dist))

                    #print("distance ",dist)

                    normal_vec = (partner_point-x)/dist
                    #print("normal_vec ",normal_vec)
                    vel_towards = np.dot(v,normal_vec) # CORRECT VELOCITY!!!!
                    #print("vel towards ",vel_towards)


                    force_towards = self.k_value * (dist-default_length) + self.damp_value * vel_towards

                    
                    #print("curr_total_force: ",force_towards)
                    main_force+= normal_vec*force_towards

            main_accel = main_force/self.mass + np.array([0,self.gravity])
        #print("accel: ",main_accel)
        #exit()
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
        #print(self.P_list)
        for main_idx, main_point in enumerate(self.P_list):
            #print(main_point)
            main_coord = main_point/self.display_size*512+256
            #print(main_coord)
            main_coord = np.array(main_coord, dtype=np.int32)
            

            for partner_idx, partner_point in enumerate(self.P_list):

                partner_coord = self.P_list[partner_idx]/self.display_size*512+256
                partner_coord = np.array(partner_coord, dtype=np.int32)
                #print("LINE: ",main_idx,partner_idx,"BOOL",self.length_connections_matrix[main_idx][partner_idx])
                default_length = self.length_connections_matrix[main_idx][partner_idx]
                
                if default_length!=0.0 and default_length!=None and not np.isnan(default_length):

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

        for main_idx, main_point in enumerate(self.P_list):
            main_coord = self.P_list[main_idx]/self.display_size*512+256
            main_coord = np.array(main_coord, dtype=np.int32)
            canvas=cv2.circle(canvas, main_coord, 4, (.1, .1, .9), -1)
            
            kinetic_energy_node+=0.5*np.sum(self.V_list[main_idx]**2)*self.mass/self.num_points

            potential_energy_node+=self.mass*self.gravity*(self.ground_value-self.P_list[main_idx, 1])/self.num_points

        TOTAL_NODE_ENERGY=kinetic_energy_node+potential_energy_node
        GROUND_DISPLAY_COORDS=int(self.ground_value/self.display_size*512+256)

        
        
        canvas[:GROUND_DISPLAY_COORDS] = np.array([0.4,0.4,0.4])
        canvas[:, 0:30] = np.array((.9, .9, .9))
        canvas[:int(512*0.9*(total_potential_spring)/(self.init_potential_energy/self.num_points)), 10:15] = np.array((.1, .5, .9))
        canvas[:int(512*0.9*(TOTAL_NODE_ENERGY)/(self.init_potential_energy/self.num_points)), 0:5] = np.array((.9, .5, .0))
        canvas[:int(512*0.9*(TOTAL_NODE_ENERGY+total_potential_spring)/(self.init_potential_energy/self.num_points)), 20:25] = np.array((.3, .9, .3))


        return np.flip(canvas, axis=0)


