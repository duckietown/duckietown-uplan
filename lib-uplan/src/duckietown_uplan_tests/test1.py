# coding=utf-8
import numpy as np
from comptests import comptest, run_module_tests, comptest_fails
import duckietown_uplan as uplan
import duckietown_world as dw


@comptest
def test_sim1():
    current_map = dw.load_map('4way')
    simulation_exp = uplan.ConstantProbabiltiySim(current_map, 3)
    simulation_exp.execute_simulation(5)

#    assert 0.1 + 0.2 != 0.3


# @comptest
# def test_sum2():
#     np.testing.assert_almost_equal(0.1 + 0.2, 0.3)


# use comptest_fails for a test that is supposed to fail
# @comptest_fails
# def test_supposed_to_fail():
#     raise Exception()


if __name__ == '__main__':
    run_module_tests()
