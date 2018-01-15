import numpy as np

def make_interaction_matrix(x, y, Zc): 
	return np.array([[-1/Zc, 0, x/Zc, x*y, -(1+x**2), y], 
					 [0, -1/Zc, y/Zc, 1+y**2, -x*y, -x]])

