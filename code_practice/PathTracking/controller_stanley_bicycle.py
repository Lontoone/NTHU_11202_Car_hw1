import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller
def angle_norm(theta):
    return (theta + 180) % 360 - 180
class ControllerStanleyBicycle(Controller):
    def __init__(self, kp=0.5):
        self.path = None
        self.kp = kp

    # State: [x, y, yaw, delta, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v, l = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"]

        # Search Front Wheel Target
        front_x = x + l*np.cos(np.deg2rad(yaw))
        front_y = y + l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta))
        min_idx, min_dist = utils.search_nearest(self.path, (front_x,front_y))
        target = self.path[min_idx]

        # TODO: Stanley Control for Bicycle Kinematic Model
        if(min_idx ==0):
            min_idx = 1
        if(min_idx == len( self.path) -1 ):
            return 0 , target
        

        theta_p = np.arctan2( self.path[min_idx+1,1] - self.path[min_idx ,1] , self.path[min_idx+1,0]- self.path[min_idx,0] )
        ang =   np.deg2rad(yaw)
        theta_e = theta_p - ang

        e_xy = np.array([ x -  self.path[min_idx,0] , y - self.path[min_idx,1] ])
        
        e = np.matmul( 
                e_xy.reshape(1,-1)  , 
                np.array([np.cos(theta_p  +np.deg2rad(90) ) , np.sin(theta_p  +np.deg2rad(90) )] ).reshape(-1,1)
        )
        
        phi = np.arctan((-self.kp * e[0,0] )/ (vf+0.001 )) + theta_e
        next_delta = angle_norm(np.rad2deg(phi))

        return next_delta, target
