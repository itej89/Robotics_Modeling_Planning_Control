import math

class mass_properties:
    def __init__(self, body_ID, mass, CM_location):
        self.body_id  =body_ID
        self.mass = mass
        self.cm_location = CM_location

class contact_description:
    def __init__(self, body_ID,contact_body_ID,location,normal_direction,friction_coefficient):
        self.body_id = body_ID
        self.contact_id = contact_body_ID
        self.location = location
        self.theta_normal = normal_direction
        self.friction_coefficient = friction_coefficient
        self.alpha_wrench = math.atan(friction_coefficient)