from compas.geometry import Transformation
from compas.geometry import Translation


class WorkpieceManager:
    """Manages the timing and viewer caching for dynamic workpieces."""
    
    def __init__(self):
        self.meshes = {}
        self.rules = {}
        self.latest_stock_vanish_time = 0.0
        self.lumber_yard_stock_y = 0.0

    def add_stock(self, name, mesh, attach_time, attachment_frame):
        self.lumber_yard_stock_y += 0.30  
        T_park = Translation.from_vector([-1.0, self.lumber_yard_stock_y, 0.05])
        
        self.meshes[name] = mesh
        self.rules[name] = {
            "attach_time": attach_time,
            "detach_time": float('inf'),
            "T_grasp": Transformation.from_frame(attachment_frame),
            "T_park": T_park,
            "vanish_delay": 4.0,
            "appear_time": max(0.0, attach_time - 3.0)
        }

    def drop_stock(self, name, detach_time):
        if name in self.rules:
            self.rules[name]["detach_time"] = detach_time
            delay = self.rules[name].get("vanish_delay", 0.0)
            
            self.latest_stock_vanish_time = detach_time + delay

    def add_element(self, name, mesh, attach_time, attachment_frame):
        
        self.meshes[name] = mesh
        self.rules[name] = {
            "attach_time": attach_time,
            "detach_time": float('inf'),
            "T_grasp": Transformation.from_frame(attachment_frame),
            "appear_time": self.latest_stock_vanish_time
        }

    def drop_element(self, name, detach_time):
        if name in self.rules:
            self.rules[name]["detach_time"] = detach_time