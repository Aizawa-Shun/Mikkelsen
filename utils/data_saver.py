import csv
import os

class DataSaver:
    def __init__(self, filepath):
        self.filepath = filepath
        if not os.path.exists(filepath):
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['epoch', 'reward'])

    def save_data(self, epoch, reward):
        with open(self.filepath, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([epoch, reward])
