import matplotlib.pyplot as plt
import csv

class Plotter:
    @staticmethod
    def plot_data(filepath):
        epochs = []
        rewards = []
        with open(filepath, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header
            for row in reader:
                epochs.append(int(row[0]))
                rewards.append(float(row[1]))

        plt.plot(epochs, rewards)
        plt.xlabel('Epoch')
        plt.ylabel('Reward')
        plt.title('Training Progress')
        plt.savefig('data/training_data.png')
        plt.show()
