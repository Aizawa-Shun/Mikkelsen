from utils.config import load_config
from simulation.environment import Environment
from simulation.agent import DQNAgent
from simulation.training import train_agent

def run_dql_simulation(config, mode):
    environment = Environment(config['simulation'], mode)
    agent = DQNAgent(config['dql_agent']['input_size'], config['dql_agent'])
    train_agent(environment, agent, config['dql_agent'])
    environment.disconnect()

def run_test_simulation(config, mode):
    environment = Environment(config['simulation'], mode)
    environment.disconnect()

def main():
    config = load_config('configs/config.yaml')

    mode = config['mode']

    if mode == 'simulation2d' or 'simulation3d':
        run_dql_simulation(config, mode)
    elif mode =='simulation_test':
        run_test_simulation(config, mode)
    else:
        print(f'not found {mode} mode.')
    
    print('finish.')

if __name__ == '__main__':
    main()
