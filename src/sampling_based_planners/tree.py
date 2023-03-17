class Node:

    def __init__(self,q,u=None,w=None):
        """ q = {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta} """
        self.q = q
        self.u = u
        self.w = w
        self.children = []
        self.parent = None

    def add_child(self,child_x):
        child_x.parent = self
        self.children.append(child_x)
    
    def remove_child(self,child_x):
        self.children = [child for child in self.children if child is not child_x] 
    
    def add_weight(self,w):
        self.w = w    
        
    def change_parent(self,to):
        if self.parent != None:
            self.parent.remove_child(self)
        to.add_child(self)
        self.parent = to
