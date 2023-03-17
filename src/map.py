import numpy as np
import cv2 as cv
from .control2d import Control2D
from .cfg import params

class Map:
    def __init__(self,size,q_start, q_goal,
                obstacles_color= (0,255,0,255),end_points_colors=(255,0,0),
                tree_color=(0,0,0),path_color=(255,0,0)):
        
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        
        """
        self.size = size
        self.q_start = q_start
        self.q_goal =  q_goal
        self.end_points_raduis = 5
        self.ends_color = tuple(reversed(end_points_colors))
        self.obstacles_color = tuple(reversed(obstacles_color))
        self.tree_color = tuple(reversed(tree_color))
        self.path_color = tuple(reversed(path_color))
        self.map_img = np.ones(size,np.uint8)*255
        self.fresh_map()
        
    
    def fresh_map(self):
        self.map_img[:][:][:] = 255
        self.draw_end_points()
        self.draw_obstacles()
        
        
    def draw_end_points(self):
        cv.circle(self.map_img,center=(self.q_start["x"],self.q_start["y"]),radius=self.end_points_raduis,color=self.ends_color,thickness=-1,lineType=8)
        cv.circle(self.map_img,center=(self.q_goal["x"],self.q_goal["y"]),radius=self.end_points_raduis,color=self.ends_color,thickness=-1,lineType=8)

    def draw_obstacles(self):
        #draw obstacles
        color = self.obstacles_color
        cv.rectangle(self.map_img,(255,255),(300,300),color,thickness=-1)
        cv.rectangle(self.map_img,(150,150),(200,200),color,thickness=-1)
        cv.rectangle(self.map_img,(100,250),(170,300),color,thickness=-1)
        cv.rectangle(self.map_img,(100,80),(150,120),color,thickness=-1)
        cv.rectangle(self.map_img,(10,80),(50,120),color,thickness=-1)
        cv.rectangle(self.map_img,(210,320),(240,350),color,thickness=-1)
        cv.rectangle(self.map_img,(10,180),(60,220),color,thickness=-1)
        cv.circle(self.map_img,center=(340,60),radius=40,color=color,thickness=-1)
        cv.circle(self.map_img,center=(250,60),radius=40,color=color,thickness=-1)
        
    def draw_path(self,path,width=2):
        cntrl = Control2D(path[0][0])
        for q1,q2 in path:
            cntrl.update_state(q_state=q1)
            line = cntrl.steer(q_desired=q2,v_desired=params["v_max"],steps=params["step_samples"])
            self.draw_line(line,color=self.path_color,width=width)
            
    def erase_path(self,path,width=2):
        if len(path) == 0:
            return
        cntrl = Control2D(path[0][0])
        for q1,q2 in path:
            cntrl.update_state(q_state=q1)
            line = cntrl.steer(q_desired=q2,v_desired=params["v_max"],steps=params["step_samples"])
            self.draw_line(line,color=(255,255,255),width=width)   
            self.draw_line(line,color=self.tree_color,width=1)   
        
    def draw_line(self,line,color,width=1):
        len_ = len(line)
        for i in range(len_):
            if i == len_-1:
                break
            self.draw_branch(line[i],line[i+1],color,width)
    
    def draw_branch(self,q1, q2,color,width = 1):
        # Drawing line
        cv.line(self.map_img,pt1=(q1["x"],q1["y"]),pt2=(q2["x"],q2["y"]),color=color,thickness=width)
    
    def draw_point(self,q_point,raduis,color):
        # Drawing point
        cv.circle(self.map_img,center=(q_point["x"],q_point["y"]),radius = raduis,color=color,thickness=-1)
        
    def draw_ellipse(self,q_start,q_goal,c_max,c_min,Q_center,color=(0,0,255)):
        a = np.sqrt(c_max ** 2 - c_min ** 2) 
        b = c_max 
        angle = np.arctan2((q_goal["y"] - q_start["y"]),(q_goal["x"] - q_start["x"]))*180/np.pi
        cx = round(Q_center[0][0])# - a/2.0
        cy = round(Q_center[1][0])# - b / 2.0
        cv.ellipse(img=self.map_img,center=(cx,cy),axes=(round(b/2),round(a/2)),angle=angle, startAngle=0,endAngle=360,
                    color=tuple(reversed(color)),
                    thickness=1,
                    lineType=8)

    def show_map(self,name):
        cv.imshow(name,self.map_img)
        if cv.waitKey(1) == ord('q'):
            return