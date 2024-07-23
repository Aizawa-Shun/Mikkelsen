import matplotlib.pyplot as plt
import csv


class Plotter:
    @staticmethod
    def plot_rewards(filepath):
        episodes = []
        rewards = []
        with open(filepath, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header
            for row in reader:
                episodes.append(int(row[0]))
                rewards.append(float(row[1]))

        plt.plot(episodes, rewards)
        plt.xlabel('Episodes')
        plt.ylabel('Reward')
        plt.title('Training Progress')
        plt.savefig('data/rewards.png')
        plt.show()

    @staticmethod
    def plot_joint_angles(angles_filepath, episode_to_plot):
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
        plt.savefig(f'data/joint_angles_episode_{episode_to_plot}.png')
        plt.show()