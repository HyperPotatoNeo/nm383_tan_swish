import numpy as np
import rospy

#DECAY PARAMETERS
EPSILON = 10e-5
LIN_DECAY = 0
LIN_BIAS = 0.0
EXP_DECAY = 0.0
EXP_BIAS = 0.0

#RBF PARAMETERS
LIN_ALPHA = 1000
QUAD_GAMMA = 100
GAUSS_SIGMA = 1
LIN_LOG = 1000

INITIAL_FLAG = True
WEIGHTAGE = 0.05

class Radial_Basis_Function():
	@staticmethod
	def linear(r):
		z = LIN_ALPHA/(r+EPSILON)
		return z

	@staticmethod
	def quadratic(r):
		z = QUAD_GAMMA/(r**2+EPSILON)
		return z

	@staticmethod
	def gaussian(r):
		z = np.exp(-(r**2)/(GAUSS_SIGMA**2))
		return z

	@staticmethod
	def loglinear(r):
		z = LIN_LOG/(r*np.log(r+EPSILON) + EPSILON)

def decay(x, k):
	if(k==0):
		x -= LIN_DECAY*x + LIN_BIAS
	elif(k==1):
		x -= np.exp(-EXP_DECAY*x) + EXP_BIAS
	return x
        

def drone_potential(pot_grid, center_state):
	global INITIAL_FLAG
	#rospy.loginfo(center_state)
	for i in range(pot_grid.shape[0]):
		if (i-center_state[0] > 50):
			continue;
		for j in range(pot_grid.shape[1]):
			if (j-center_state[1] > 50):
				continue
			# print(center_state)
			r = np.sqrt((i-center_state[0])**2 + (j-center_state[1])**2)
			if(INITIAL_FLAG):
				pot_grid[i,j,0] = Radial_Basis_Function.linear(r)
				pot_grid[i,j,0] = np.min([pot_grid[i,j,0], 255])
			
			else:
				true_p = (1 - WEIGHTAGE)*Radial_Basis_Function.linear(r) + WEIGHTAGE*pot_grid[i,j,0]
				if(true_p>pot_grid[i,j,0]):
					pot_grid[i,j,0] = true_p #ADD MOMENTUM OR OTHER SECOND ORDER UPDATE RULE
					pot_grid[i,j,0] = np.min([pot_grid[i,j,0], 255])

				else:
					pot_grid[i,j,0] = (WEIGHTAGE)*Radial_Basis_Function.linear(r) + (1-WEIGHTAGE)*pot_grid[i,j,0]

				if(center_state[0]==i and center_state[1]==j):
					pot_grid[i,j,0] = 255
	#print(pot_grid[:,:,0])
	INITIAL_FLAG = False
	return pot_grid

def obstacles_potential(pot_grid, center_state):
	for i in range(pot_grid.shape[0]):
		for j in range(pot_grid.shape[1]):
			#print('****************************')
			#print(center_state)
			r = np.sqrt((i-center_state[0])**2 + (j-center_state[1])**2)
			pot_grid[i,j,1] = Radial_Basis_Function.linear(r)
			pot_grid[i,j,1] = np.min([pot_grid[i,j,1], 255])
	#print(pot_grid[:,:,1])
	return pot_grid

def recharge_potential(pot_grid, center_state, agent_list):
	for i in range(len(agent_list)):
		index = agent_list[i].index
		for j in range(pot_grid.shape[0]):
			for k in range(pot_grid.shape[1]):
				if(agent_list[i].battery>30):
					r = np.sqrt((j-center_state[0])**2 + (k-center_state[1])**2)
					pot_grid[j,k,index] += Radial_Basis_Function.linear(r)*0.05#*(100 - agent_list[i].battery)  NEED TO TRY OUT BATTERY PARAMETERS WITH REAL AGENT
					pot_grid[j,k,index] = np.min([pot_grid[j,k,index], 255])
				
				else:
					r = np.sqrt((j-center_state[0])**2 + (k-center_state[1])**2)
					pot_grid[j,k,index] = Radial_Basis_Function.linear(r)
					pot_grid[j,k,index] = np.min([pot_grid[j,k,index], 255])
	return pot_grid

def potential_compute(pot_grid, pos_grid, FIELD_GENERATORS, agent_list, time_passed):
	t_pot_grid = pot_grid
	#print(type(pot_grid.shape))
	for i in range(pot_grid.shape[0]):
		for j in range(pot_grid.shape[1]):
			for k in range(pot_grid.shape[2]):
				if(np.abs(pot_grid[i,j,k])>EPSILON):
					pass
					pot_grid[i,j,k] = decay(pot_grid[i,j,k], k)
	
	for i in range(len(FIELD_GENERATORS['drones'])):
		center_state = FIELD_GENERATORS['drones'][i]
		# print('drones',center_state)
		pot_grid = drone_potential(pot_grid, center_state)
	

	if (time_passed == 0):
		for i in range(len(FIELD_GENERATORS['obstacles'])):
			center_state = FIELD_GENERATORS['obstacles'][i]
			# print('obstacles',center_state)
			pot_grid = obstacles_potential(pot_grid, center_state)

	if (time_passed > 0 and time_passed % 4 == 0):
		for i in range(len(FIELD_GENERATORS['recharge'])):
			center_state = FIELD_GENERATORS['recharge'][i]
			# print('recharge',center_state)
			pot_grid = recharge_potential(pot_grid, center_state, agent_list)

	#print('drones', pot_grid[:,:,0])
	#print('recharge', pot_grid[:,:,2])
	return pot_grid