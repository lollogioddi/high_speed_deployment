import os
import pandas as pd

# Specify the folder containing the CSV files
csv_folder_path = '/home/lollogioddi/Documents/thesis/stesura/raccolta_dati_finale_2/ideal_12sett/adaptive/csv'  # Replace with your actual folder path

# Loop through each CSV file in the folder
for filename in os.listdir(csv_folder_path):
    if filename.endswith('.csv'):  # Ensure we are only processing CSV files
        file_path = os.path.join(csv_folder_path, filename)
        
        # Load the CSV file without headers
        df = pd.read_csv(file_path, header=None)
        
        # Add "/time" to the first cell (A1)
        df.iloc[0, 0] = '/time'
        
        # Save the updated DataFrame back to the same CSV file without headers
        df.to_csv(file_path, header=False, index=False)

        print(f"Updated {filename} with '/time' in the first cell.")

print("All files processed.")

