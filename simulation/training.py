from utils.data_saver import DataSaver
from utils.plotter import Plotter

data_saver = DataSaver('data/training_data.csv')

def train_agent(environment, agent, config):
    target_reward = config['target_reward']

    for episode in range(config['episodes']):
        if episode != 0: states = environment.reset()
        agent.setup()
        environment.step_simulation()
        environment.get_observation()
        agent.first_states(environment.states)
        done = False
        states = environment.states

        while not done:
            action, one_hot = agent.act()
            reward, done = environment.step(action)
            next_state = environment.states
            agent.learn(states, action, reward, one_hot, next_state, done)
            states = next_state
        
        # Calculate average reward
        average_reward = agent.calculate_average_reward()
        # Save data
        data_saver.save_data(episode, average_reward)
        
        if episode % agent.targget_update == 0:
            agent.update_store_data()
            agent.update_policy()
            average_reward = agent.calculate_average_reward()
            print(f'[{episode} episode] average reward: {average_reward:.2f}')

            # Finish learning
            if average_reward >= target_reward:
                agent.save_weight_file()
                Plotter.plot_data('data/training_data.csv')