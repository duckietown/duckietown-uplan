"""
This will have our main file
"""
import contracts
contracts.disable_all()
import duckietown_uplan as uplan
import duckietown_world as dw


def main():
    current_map = dw.load_map('4way')
    simulation_exp = uplan.ConstantProbabiltiySim(current_map, 1)
    simulation_exp.execute_simulation_video(50)


if __name__ == "__main__":
    main()