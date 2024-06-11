import time
import matplotlib.pyplot as plt
import cv2
import numpy as np

dt=0.001

# torque = angular_acceleration * moment_of_inertia
# torque = r x f

class rigid_body:
    
    def __init__(self, gravity, display_size, display_resolution):
        self.gravity = gravity
        self.display_size = display_size
        self.display_resolution = display_resolution
        self.object_list = []
    

    def add(self, object):
        self.object_list.append(object)
    
    def step(self):

        pass

class box:

    def __init__(self, center_of_mass_position, width, height, mass, init_angle):

        self.pos = center_of_mass_position
        self.angle = init_angle

        self.vel = np.zeros([0.0, 0])
        self.vel_angle = np.zeros([0.0])

        

        self.width = width
        self.height = height
        self.mass = mass
        self.inertia = mass*(height**2+width**2)/12


