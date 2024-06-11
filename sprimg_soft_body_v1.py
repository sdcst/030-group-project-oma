import matplotlib.pyplot as plt
import cv2
import numpy as np

dt=0.0001

class spring:

    def __init__(self, p_list, length_matrix, mass, locklist=None):
        
        self.num_of_points = p_list.shape[0]
        # length_matrix is (num_of_points, num_of_points)

        self.k_value  = 15
        self.damp_value = 10

        self.p_list = p_list
        self.v_list = np.zeros(p_list.shape)

        self.length_matrix = length_matrix


        self.display_size = 32

        if locklist==None:
            locklist = np.zeros(p_list.shape[0], dtype=bool)
            locklist[0] = True
        
        self.locklist = locklist
    
    def step(self):

        for point, idx in enumerate(self.p_list):

            if self.locklist[idx]

            for ref_point in self.length_matrix:

                dist = np.sqrt(np.sum((self.p0-self.p1)**2))
                vel_towards = np.sqrt(np.sum((self.v0-self.v1)**2))

                self.force_towards = self.k_value/self.mass_0 * (dist-self.length) + self.damp_value/self.mass_0 * vel_towards

                normal_vec = (self.p0-self.p1)/dist

                force_vec = normal_vec*self.force_towards + np.array([0, -19])*self.mass_0

                self.v1 += force_vec*dt/self.mass_0
                self.p1 += self.v1*dt

        