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
        
        if episode % agent.targget_update == 0:
            agent.update_store_data()
            agent.update_policy()
            print(f'[{episode} episode] average reward: {agent.average_reward:.2f}')
            if agent.average_reward >= target_reward:
                agent.save_weight_file()