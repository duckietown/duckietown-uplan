import numpy as np
from comptests import comptest, run_module_tests, comptest_fails
import duckietown_uplan as uplan
import duckietown_world as dw

SIM_TIME = 15
NUM_DUCKIES = 5
MAP_TYPE = '4way'

@comptest
def test_sim1():
    current_map = dw.load_map(MAP_TYPE)
    simulation_exp = uplan.ConstantProbabiltiySim(current_map, NUM_DUCKIES)
    simulation_exp.execute_simulation(SIM_TIME)



if __name__ == '__main__':
    run_module_tests()
