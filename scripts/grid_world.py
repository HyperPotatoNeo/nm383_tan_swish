import numpy as np
import potential_func
import agent

NUM_FIELDS = 3

def create_grid(h, w, num_agents, init_states, num_obstacles=0, obstacles=(), num_stations=0, stations=()):
	#Position grid 0: drones 1: obstacle 2: recharge_station
	#Potential grid 0: drone_cover 1: obstacle 2: recharge_station

	global NUM_FIELDS

	position_grid = np.zeros((h,w,3))
	potential_grid = np.zeros((h,w,NUM_FIELDS))

	for i in range(num_agents):
		position_grid[init_states[i], 0] = 1
	for i in range(num_obstacles):
		position_grid[obstacles[i], 1] = 1
	for i in range(num_stations):
		position_grid[stations[i], 2] = 1

	return position_grid, potential_grid
