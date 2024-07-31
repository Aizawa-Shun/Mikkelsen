from utils.config import load_config
from simulation.environment import Environment
from simulation.dql.agent import DQNAgent
from simulation.dql.training import train_agent
from real_robot.controller import HardwareController

def run_train_dql(config, operation, mode, dimension, target_action, algorithm):
    environment = Environment(config['simulation'], operation, mode, dimension, target_action)
    agent = DQNAgent(config['dql_agent']['input_size'], config['dql_agent'], dimension)
    train_agent(environment, agent, config['dql_agent'], dimension, mode, algorithm)
    environment.disconnect()

def run_train_ars(config, operation, mode, dimension, target_action, algorithm):
    environment = Environment(config['simulation'], operation, mode, dimension, target_action)
    agent = DQNAgent(config['ars_agent']['input_size'], config['ars_agent'], dimension)
    train_agent(environment, agent, config['dql_agent'], mode, algorithm)
    environment.disconnect()

def run_footstep():
    pass

def run_test_simulation(config, operation, mode, dimension, target_action):
    environment = Environment(config['simulation'], operation, mode, dimension, target_action)
    environment.disconnect()

def run_test_real_robot(config, operation):
    controller = HardwareController(config['real_robot'], operation)

def main():
    config = load_config('configs/config.yaml')

    operation = config['operation']
    mode = config['mode']
    algorithm = config['algorithm']
    target_action = config['target_action']
    dimension = config['dimension']

    if operation == 'simulation':
        if mode == 'train':
            if algorithm == 'dql':
                run_train_dql(config, operation, mode, dimension, target_action, algorithm)
            elif algorithm == 'ars':
                pass
        elif mode == 'execution':
            pass
        elif mode == 'footstep':
            run_footstep()
        elif mode == 'test':
            run_test_simulation(config, operation, mode, dimension, target_action)
    elif operation == 'real_robot':
        run_test_real_robot()
    else:
        print(f'not found {operation}.')
    
    print('finish.')

if __name__ == '__main__':
    main()
