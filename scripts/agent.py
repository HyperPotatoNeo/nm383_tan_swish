import numpy as np
import potential_func

class Agent():
	def __init__(self, initial_pos, index):
		self.index = index
		self.pos = initial_pos
		self.battery = 100

	def agent_step(self, pos_grid, pot_grid, field_generators):
		least_step, ret = potential_func.scan_grad(self.pos, pot_grid)
		temp = self.pos

		if(ret != -1):
			self.pos += least_step
			field_generators['drones'][self.index] = self.pos
			pos_grid[temp, 0] = 0
			pos_grid[temp+least_step, 0] = 1
			#grad_compute(pot_grid, pos_grid)
			return pos_grid, pot_grid

def create_agent(num_agents, init_state):
	agent_list = []
	for i in range(num_agents):
		agent_list.append(Agent(init_state[i], i))

	return agent_list

