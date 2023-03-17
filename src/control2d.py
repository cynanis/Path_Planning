import numpy as np
from cfg import params

class Control2D:
    def __init__(self,q_init):
        """ q = {"x":x,"y":y,"theta":theta,"delta":delta,"beta":beta} """
        self.q_state = q_init
        
    def update_state(self,q_state):
        self.q_state = q_state    
        
    def forward(self, v, delta):
        ## BICYCLE FORWARD MODEL
        #w steering angle rate  
        #v bicycle speed
        # ==================================
        if v > 0:
            v = min(v,params["v_max"])  
        else:
            v = max(v,-params["v_max"])  
        
        if delta >0:
            delta = min(delta,params["steer_max"])
        else:
            delta = max(delta,-params["steer_max"])
        #print("q_nearest: ",self.q_state)
        xc,yc,theta,delta_,beta =self.q_state.values()
        #setup the next states using the differential equations     
        xc = xc + (v*np.cos(theta + beta))*params["sample_time"]
        yc  = yc + (v*np.sin(theta + beta))*params["sample_time"]
        theta = theta + (v*np.cos(beta)*np.tan(delta_)/params["L"]) * params["sample_time"]
        #delta = delta + w * params["sample_time"]
        beta = np.arctan(params["lr"] * np.tan(delta) / params["L"])
        # ==================================
        q_new = {"x":round(xc),"y":round(yc),"theta":theta,"delta":delta,"beta":beta}
        return q_new

    
    
    def lateral_control_stanly(self,q_desired,v):
            # Stanly Controller
            xd,yd,theta = q_desired["x"],q_desired["y"],q_desired["theta"]
            xc,yc,theta_c = self.q_state["x"],self.q_state["y"],self.q_state["theta"]
            
            ### HEADING ERROR ###
            #path equation aX + bY + c = 0 => Y = -a/b*X -c
            #path slop (Yf - Yi)/(Xi - Xf)
            path_slop = np.tan(theta)

            #heading of the path
            path_heading = theta
    
            #vehicle heading
            vehicle_heading = theta_c

            #(yaw angle) heading of the vehicle with respect to the path
            heading_error = path_heading - vehicle_heading  
            if heading_error > np.pi:
                heading_error -= 2*np.pi
            elif heading_error < -np.pi:
                heading_error += 2*np.pi
            #print("heading error {}".format(heading_error))
                
            # #CROSSTRACK ERROR
            k_err = params["kp_crss"]
            
            a = -path_slop
            b = 1.0
            c = (path_slop*xd) - yd
        
            
            # cross track error 
            crosstrack_error = (a*xc + b*yc + c)/(np.sqrt(a**2 + b**2))
            vehicle_path_angle = np.arctan2(yc-yd, xc-xd)
            path_to_vehicle_diff = path_heading - vehicle_path_angle
            if path_to_vehicle_diff > np.pi:
                path_to_vehicle_diff -= 2 * np.pi
            if path_to_vehicle_diff < - np.pi:
                path_to_vehicle_diff += 2 * np.pi
            if path_to_vehicle_diff > 0:
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = - abs(crosstrack_error)
            #print("cross_error {}".format(crosstrack_error))
            
            cross_track_steering = np.arctan(k_err*crosstrack_error/v)
            #print("steer_cross_track {}".format(cross_track_steering))

            # Change the steer output with the lateral controller. 
            steer = heading_error +  cross_track_steering        
            if steer > np.pi:
                steer -= 2*np.pi
            elif steer < -np.pi:
                steer += 2*np.pi
    
            #print("steer {}".format(steer))
            return steer
    
    def longitudinal_control_pid(self,v_desired):
        return v_desired

    def steer(self,q_desired,v_desired,steps):
        line = [self.q_state]

        for i in range(steps):
            self.update_state(q_state = line[i])
            v = self.longitudinal_control_pid(v_desired=v_desired)
            delta = self.lateral_control_stanly(q_desired,v)
            q_new = self.forward(v,delta)
            line.append(q_new)
        return line


# def pure_pursuit(q_state,q_d,v):
#         xd,yd,theta = q_d["x"],q_d["y"]
#         # steering angle rate pure pursuit control
#         # alpha: look ahead direction
#         alpha = np.arctan((yd - q_state["y"])/(xd - q_state["x"]+1e-7)) - q_state["y"]
#         delta = np.arctan((2*params["L"]*np.sin(alpha))/(params["kp_ld"]*v))
#         #w = (delta_d - self.q_state["delta"])/params["sample_time"]
#         return v, delta