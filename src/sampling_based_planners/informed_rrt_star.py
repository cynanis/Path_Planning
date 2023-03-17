import numpy as np
from random import randint
import sys
from .tree import Node
from .rrt_star import RRTStar

sys.path.append('./src')
from cfg import params

class InformedRRTStar(RRTStar):
    def __init__(self,map,obstacle_clearence=3,goal_threshold=8, rewire_radius = 6, name="Informed RRT*"):        
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        """
        super().__init__(map,obstacle_clearence,goal_threshold,rewire_radius,name)
        self.sample_config = self.hyperellipsoid_config()
        
    def build(self,steps):
        X_soln = set()
        for i in range(steps):
            #the transverse diameter is cbest of the special hyperellipsoid
            x_goal , c_best = self.transverse_diameter(X_soln)
            #get a random location sample in the map
            q_rand = self.sample(c_best)
            #find the nearest node to the random sample
            _,x_nearest = self.nearest_neighbor(self.tree,q_rand)
            #genertate a steering trajectory from nearest node to random sample
            q_new, line_new  = self.steer(x_nearest.q,q_rand)
            
            #check for collision
            if self.collision_free(line_new): 
                x_new = Node(q_new)

                # find list of x nodes that lie in the circle centerd at q_new
                r = self.search_raduis(self.tree,self.rewire_radius)
                X_near = self.near(self.tree,q_new,r)
                
                 #connect x_new to x_near with minimal cost
                self.connect(X_near,x_new,x_nearest,line_new)
                        
                #rewire the tree 
                for x_near in X_near:
                    self.rewire(x_near,x_new,self.sample_config,x_goal)

                #check goal region
                if self.in_goal_region(x_new.q):
                    X_soln.add(x_new)
                    self.update_path(X_soln,x_new)
        
                if i%20 == 0:
                    self.map.show_map(self.name)
                

    
    def transverse_diameter(self,X_soln):
        c_best = np.inf
        x_best = None
        if X_soln:
            costs = {node: self.cost(node) for node in X_soln}
            x_best = min(costs, key=costs.get)
            c_best = costs[x_best]    
        return x_best,c_best
        
    def sample(self,c_max):
        if c_max < np.inf:
            c_min, Q_center, C = self.sample_config
            r = [c_max / 2.0,
                 np.sqrt(c_max ** 2 - c_min ** 2) / 2.0,
                 np.sqrt(c_max ** 2 - c_min ** 2) / 2.0]
            L = np.diag(r)
            while True:
                Q_ball = self.sample_unit_nball()
                Q_rand = np.dot(np.dot(C,L),Q_ball) + Q_center
                q_rand = {"x":round(Q_rand[0][0]),"y":round(Q_rand[1][0]),"theta":np.random.uniform(-np.pi,np.pi)}
                if self.point_collision_free(q_rand):
                    return q_rand 
        else:
            return self.sample_free()    

    def rotaion_to_world_frame(self):
        a1 = np.array([
                        [(self.map.q_goal["x"]-self.map.q_start["x"])/self.ecludian(self.map.q_goal,self.map.q_start)],
                        [(self.map.q_goal["y"]-self.map.q_start["y"])/self.ecludian(self.map.q_goal,self.map.q_start)],
                        [0.0],
                    ])
        I1 = np.array([[1.0],[0],[0]])
        M = np.dot(a1,I1.T)
        U,_,V_T = np.linalg.svd(M)
        C = np.dot(np.dot(U,np.diag([1.0,1.0,np.linalg.det(U)*np.linalg.det(V_T.T)])),V_T)
        return C
    
    def sample_unit_nball(self):
        while True:
            x , y = np.random.uniform(-1,1),np.random.uniform(-1,1)
            if x**2 + y**2 < 1.0:
                return np.array([[x],[y],[0.0]])
    
    def hyperellipsoid_config(self):
            c_min = self.ecludian(self.map.q_goal,self.map.q_start)
            Q_center = np.array([[(self.map.q_start["x"]+self.map.q_goal["x"]) / 2.0],
                            [(self.map.q_start["y"]+self.map.q_goal["y"]) / 2.0], [0.0]])
            C = self.rotaion_to_world_frame()
            return (c_min,Q_center,C)
        
    def update_path(self,X_soln,x_new):
        # get node goal with lowest cost
        x_best,c_best = self.transverse_diameter(X_soln)
        # if x_new is lowest cost node goal draw path
        if x_best == x_new:
            #erase prev path
            self.map.draw_point(self.map.q_goal,raduis=self.map.end_points_raduis,color=self.map.path_color)
            #erase old path
            self.map.erase_path(self.path,width=2)
            #extract the new path from new goal
            self.path = self.extract_path(x_new)   
            #draw the new path
            print("===> draw new path")
            self.map.draw_path(self.path,width=2)
            cmin, Q_center, _ = self.sample_config
            self.map.draw_ellipse(self.map.q_start,self.map.q_goal,c_best,cmin,Q_center)
    
  