import numpy as np
from random import randint
from .tree import Node
from .rrt_star import RRTStar
from .cfg import params
import copy

class GrowQRRT(RRTStar):
    def __init__(self,map,obstacle_clearence=3,goal_threshold=8, rewire_radius = 6, name="Grow QRRT*"):        
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        """
        super().__init__(map,obstacle_clearence,goal_threshold,rewire_radius,name)
        
    def build(self,steps):

        X_soln = set()
        for i in range(steps):
            #the transverse diameter is cbest of the special hyperellipsoid
            _ , c_best = self.transverse_diameter(X_soln)
            q_rand = self.sample(c_best)
            #find the nearest node to the random sample
            _,x_nearest = self.nearest_neighbor(self.tree,q_rand)
            #genertate a steering trajectory from nearest node to random sample
            q_new, line_n  = self.steer(x_nearest.q,q_rand)
            
            #check for collision
            if self.collision_free(line_n): 
                # find list of x nodes that lie in the circle centerd at q_new
                r = self.search_raduis(self.tree,self.rewire_radius)
                X_near = self.near(self.tree,q_new,r)  
                 #connect the x_new node to the near x_min=x_near node that result in a minimum-cost c_min path
                x_new = Node(q = q_new)
                x_min = x_nearest
                c_min = self.cost(x_nearest) + self.c(line_n)
                for x_near in X_near:
                    _,line_n = self.steer(x_near.q,x_new.q)
                    c_new = self.cost(x_near) + self.c(line_n)
                    if self.collision_free(line_n):
                        if c_new < c_min:
                            x_min = x_near
                            c_min = c_new
                
                #update the the new node weight
                _,line_n = self.steer(x_min.q,x_new.q)
                x_new.add_weight(self.c(line_n))
                #add x_new node to the tree
                x_min.add_child(x_new)      
                          
                self.map.draw_line(line_n,width=1,color=self.map.tree_color)
                        
                #rewrite the tree 
                for x_near in X_near:
                    self.rewire(x_near,x_new)

                #check goal region
                if self.in_goal_region(x_new.q):
                    X_soln.add(x_new)
                    self.update_path(X_soln,x_new)
        
                if i%20 == 0:
                    self.map.show_map(self.name)
                
    
  