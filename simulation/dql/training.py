from utils.data_saver import DataSaver
from utils.plotter import Plotter
from utils.free_memory import free_memory
from utils.restart import restart_program

data_saver = DataSaver('data/rewards.csv', 'data/joint_angles.csv', 'data/joint_torques.csv')

def train_agent(environment, agent, config):
    target_reward = config['target_reward']
    learning_interval = config['learning_interval']
    save_path = config['weight_save_path']
    count = learning_interval
    save_data = config['save_data']
    
    for episode in range(agent.episodes, config['episodes']):
        if episode != 0: states = environment.reset()
        agent.setup()
        environment.initial_pose()
        environment.step()
        environment.get_observation()
        agent.first_states(environment.states)
        done = False
        states = environment.states
        environment.done = False

        while not done:
            if learning_interval <= count:
                action, one_hot = agent.act()
                reward, done = environment.step(action)
                next_state = environment.states
                agent.learn(states, action, reward, one_hot)
                states = next_state

                if save_data:
                    data_saver.save_joint_angles(episode, environment.t, environment.states['joint_positions'])
                    data_saver.save_torque_angles(episode, environment.t, environment.states['joint_torques'])
                count = 0
            else:
                done = environment.step()
                count += environment.dt
                
        # Calculate average reward
        average_reward = agent.calculate_average_reward()
        # Save data in CSV
        if save_data:
            data_saver.save_reward(episode, average_reward)

        if episode % agent.checkpoint_interval == 0 and episode != 0 and episode != agent.episodes:
            agent.save_checkpoint(episode)
            print(f'[INFO] Checkpoint ({episode} episodes)')
            restart_program()
        
        if episode % agent.targget_update == 0 and episode != 0:
            agent.update_store_data()
            agent.update_policy()
            average_reward = agent.calculate_average_reward()
            print(f'[{episode} episode] average reward: {average_reward:.2f}')
        
            # Finish learning
            if average_reward >= target_reward:
                agent.save_weight_file(save_path)
                if save_data:
                    Plotter.plot_rewards('data/rewards.csv')
                    Plotter.plot_joint_angles('data/joint_angles.csv', episode)
