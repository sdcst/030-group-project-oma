import matplotlib.pyplot as plt
import cv2
import numpy as np
# P = nrt/V

class enviroment:

    def __init__(self, init_pos_vert_list, dt=0.01, display_size = 12 , num_verts = 8,
                radius = 4, mass=10 , temp = 380 ,moles=0.045, rigid = True,
                spring_k_val=1, spring_b_val=0.01):
        
        self.dt = dt
        self.balloon_pressure_constant = moles*temp*8.314 # nrt constant (dosnt change througout the simulation)
        # moles are inside the ballon, its assumed that outside that balloon its a vacume
        self.balloon_volume = np.sin(2*np.pi/num_verts)*num_verts*radius**2/2 # initial volume!
        self.balloon_num_verts=num_verts
        self.display_size = display_size # acts as atomatic verts
        self.vert_mass = mass/num_verts
        self.pos_vert_list = init_pos_vert_list
        self.vel_vert_list = np.zeros((len(init_pos_vert_list), 2))
        self.num_verts=num_verts

    
    def step(self):
        Fe = -2.1 * self.vert_mass
        accel_vert= []

        for i in range(self.num_verts):
            Pn = self.pos_vert_list[i]
            Pn_minus = self.pos_vert_list[i-1]
            Pn_plus = self.pos_vert_list[(i+1)%self.num_verts]

            Pn_vel = self.vel_vert_list[i]
            Pn_vel_minus = self.vel_vert_list[i-1]
            Pn_vel_plus = self.vel_vert_list[(i+1)%self.num_verts]

            delta_vel_minus = Pn_vel-Pn_vel_minus
            delta_vel_plus = Pn_vel-Pn_vel_plus

            delta_minus = Pn-Pn_minus
            delta_plus = Pn-Pn_plus


            attraction_velocity_plus = np.dot(delta_vel_plus, delta_vel_plus)
            lambda_plus = - (np.dot(Fe, delta_plus) + self.vert_mass*attraction_velocity_plus)/(np.dot(delta_plus, delta_plus))
            force_correction_plus = lambda_plus*Pn_plus

            attraction_velocity_minus = np.dot(delta_vel_minus, delta_vel_minus)
            lambda_minus = - (np.dot(Fe, delta_minus) + self.vert_mass*attraction_velocity_minus)/(np.dot(delta_minus, delta_minus))
            force_correction_minus = lambda_minus*Pn_minus

            accel_vert.append( (force_correction_plus + force_correction_minus+Fe)/self.vert_mass)
        
        for i in range(self.num_verts):
            self.vel_vert_list[i]+=accel_vert[i]*0.0001
            self.pos_vert_list[i]+=self.vel_vert_list[i]*0.0001
            #print(self.pos_vert_list)

    def draw(self):
        canvas = np.zeros([128, 128, 3])
        temp_list_point = []
        for i in range(self.num_verts):
            point = self.pos_vert_list[i]/self.display_size*128+64

            temp_list_point.append([int(point[0]), int(point[1])])
        #print([temp_list_point])
        temp_list_point = np.array(temp_list_point)
        canvas = cv2.fillPoly(canvas, [temp_list_point], (200, 10, 10))

        return canvas

list_pos = []
for i in range(8):
    angle = 2*np.pi*i/8
    list_pos.append(np.array([np.cos(angle)*2, np.sin(angle)*2]))

test_env = enviroment(list_pos)

for t in range(0, 10000):

    test_env.step()

    if t%100 == 0:

        canvas = test_env.draw()
        print(test_env.pos_vert_list)

        cv2.imshow("ball", canvas)
        cv2.waitKey(1000)
        print("NEXT FRAME")