from utils.config import load_config
from simulation.environment import Environment
from simulation.agent import DQNAgent
from simulation.training import train_agent

def run_simulation(config):
    environment = Environment(config['simulation'])
    agent = DQNAgent(config['dqn_agent']['input_size'], 
                     config['dqn_agent'])
    train_agent(environment, agent, config['dqn_agent'])
    environment.disconnect()

def main():
    config = load_config('configs/config.yaml')

    mode = config['mode']

    if mode == 'simulation':
        run_simulation(config)
    else:
        pass

if __name__ == '__main__':
    main()
    