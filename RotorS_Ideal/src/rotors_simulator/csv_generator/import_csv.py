import csv
import os

class DataLogger:
    def __init__(self, directory='data_logs', p_values=None, d_values=None, fieldnames=[]):
        self.directory = directory
        self.fieldnames = fieldnames
        os.makedirs(self.directory, exist_ok=True)

        # Convert the P and D vectors into a string representation for the filename
        p_str = 'P' + '_'.join(map(str, p_values)) if p_values is not None else 'P_default'
        d_str = 'D' + '_'.join(map(str, d_values)) if d_values is not None else 'D_default'

        # Construct the filename and path
        filename = f"{p_str}_{d_str}.csv"
        self.filepath = os.path.join(self.directory, filename)

        # Open the file and write headers
        with open(self.filepath, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writeheader()

    def log_data(self, data):
        with open(self.filepath, 'a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow(data)
