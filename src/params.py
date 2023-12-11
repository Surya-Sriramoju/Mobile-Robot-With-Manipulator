from sympy import symbols
from math import pi

class DHParam:
   def __init__(self, d, alpha, a, theta):
    self.d = d
    self.alpha = alpha
    self.a = a
    self.theta = theta

Table_DHParam = {
    'J1' : DHParam(d=0.089159 , alpha= pi/2  , a =  0        , theta = symbols("θ1")),
    'J2' : DHParam(d=0.0      , alpha= 0     , a = -0.425    , theta = symbols("θ2")),
    'J3' : DHParam(d=0.0      , alpha= 0     , a = -0.39225  , theta = symbols("θ3")),
    'J4' : DHParam(d=0.10915  , alpha= pi/2  , a =  0.0      , theta = symbols("θ4")),
    'J5' : DHParam(d=0.09465  , alpha=-pi/2  , a =  0.0      , theta = symbols("θ5")),
    'J6' : DHParam(d=0.0823   , alpha= 0     , a =  0.0      , theta = symbols("θ6"))
}