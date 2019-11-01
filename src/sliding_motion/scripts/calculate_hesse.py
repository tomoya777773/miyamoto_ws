#!/usr/bin/env python
# Not generate pyc file

import scipy.optimize
import numpy as np
from sympy import Symbol,diff

def eigenvalue_of_hesse_matrix(po_list):
    if len(po_list) < 5:
        return np.array([1,-1])
    
    regression_func = lambda param,x,y,z: z - (param[0]*x**2 + param[1]*y**2 + param[2]*x + param[3]*y + param[4])
    param = [0, 0, 0, 0, 0]
    # print "po_list:",po_list
    optimized_param =  scipy.optimize.leastsq(regression_func, param, args=(po_list[:,0], po_list[:,1], po_list[:,2]))[0]

    x = Symbol('x')
    y = Symbol('y')

    z = optimized_param[0]*x**2 + optimized_param[1]*y**2 + optimized_param[2]*x + optimized_param[3]*y + optimized_param[4]

    hesse00 = np.array(diff(diff(z, x), x), dtype=np.float32)
    hesse01 = np.array(diff(diff(z, y), x), dtype=np.float32)
    hesse10 = hesse01.copy()
    hesse11 = np.array(diff(diff(z, y), y), dtype=np.float32)

    Hesse_matrix = np.array([[hesse00,hesse01], [hesse10,hesse11]])

    eigenvalue,v = np.linalg.eig(Hesse_matrix)

    return eigenvalue