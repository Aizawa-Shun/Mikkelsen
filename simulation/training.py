from utils.data_saver import DataSaver
from utils.plotter import Plotter

data_saver = DataSaver('data/rewards.csv', 'data/joint_angles.csv')

def train_agent(environment, agent, config):
    target_reward = config['target_reward']
    learning_interval = config['learning_interval']
    count = learning_interval

    for episode in range(config['episodes']):
        if episode != 0: states = environment.reset()
        agent.setup()
        environment.step_simulation()
        environment.get_observation()
        agent.first_states(environment.states)
        done = False
        states = environment.states

        while not done:
            if learning_interval <= count:
                action, one_hot = agent.act()
                reward, done = environment.step(action)
                next_state = environment.states
                agent.learn(states, action, reward, one_hot, next_state, done)
                states = next_state

                data_saver.save_joint_angles(episode, environment.t, environment.states['joint_positions'])
                count = 0
            else:
                done = environment.step()
                count += environment.dt

                
        # Calculate average reward
        average_reward = agent.calculate_average_reward()
        # Save data in CSV
        data_saver.save_reward(episode, average_reward)
        
        if episode % agent.targget_update == 0:
            agent.update_store_data()
            agent.update_policy()
            average_reward = agent.calculate_average_reward()
            print(f'[{episode} episode] average reward: {average_reward:.2f}')
        
            # Finish learning
            if average_reward >= target_reward:
                agent.save_weight_file()
                Plotter.plot_rewards('data/rewards.csv')
                Plotter.plot_joint_angles('data/joint_angles.csv')