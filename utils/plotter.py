import matplotlib.pyplot as plt
import csv
import os

class Plotter:
    @staticmethod
    def plot_rewards(directory):
        reward_filepath = os.path.join(directory, 'rewards.csv')
        output_filepath = os.path.join(directory, 'rewards.png')
        episodes = []
        rewards = []
        with open(reward_filepath, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header
            for row in reader:
                episodes.append(int(row[0]))
                rewards.append(float(row[1]))

        plt.plot(episodes, rewards)
        plt.xlabel('Episodes')
        plt.ylabel('Reward')
        plt.title('Training Progress')
        plt.savefig(output_filepath)
        plt.show()

    @staticmethod
    def plot_joint_angles(directory, episode_to_plot):
        angles_filepath = os.path.join(directory, 'angles.csv')
        output_filepath = os.path.join(directory, 'angles.png')
        time = []
        joint_angles = [[] for _ in range(10)]  # Assuming 10 joints
        episodes = []

        with open(angles_filepath, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)  # Skip header
            for row in reader:
                episode = int(row[0])
                if episode == episode_to_plot:
                    episodes.append(episode)
                    time.append(float(row[1]))  # Changed to float
                    for i in range(10):
                        joint_angles[i].append(float(row[i + 2]))

        # Plot joint angles
        for i in range(10):
            plt.plot(time, joint_angles[i], label=f'Joint {i}')
        
        # Add vertical lines at the start of each episode
        previous_episode = episodes[0]
        for i, episode in enumerate(episodes):
            if episode != previous_episode:
                plt.axvline(x=time[i], color='grey', linestyle='--', linewidth=0.5)
                previous_episode = episode
        
        plt.xlabel('Time')
        plt.ylabel('Joint Angles')
        plt.title(f'Joint Angles Progress for Episode {episode_to_plot}')
        plt.legend()
        plt.savefig(output_filepath)
        plt.show()