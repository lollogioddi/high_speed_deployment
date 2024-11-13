import os
import pandas as pd

# Specify the folder containing the CSV files
folder_path = '/home/lollogioddi/Documents/thesis/stesura/raccolta_dati_finale_2/ideal_12sett/adaptive/csv'  # Path to your folder containing the CSV files
output_folder_path = '/home/lollogioddi/Documents/thesis/stesura/raccolta_dati_finale_2/ideal_12sett/adaptive/csv'  # Folder to save the adjusted files

# Create output folder if it doesn't exist
os.makedirs(output_folder_path, exist_ok=True)

# Loop through each CSV file in the folder
for filename in os.listdir(folder_path):
    if filename.endswith('.csv'):  # Ensure we are only processing CSV files
        file_path = os.path.join(folder_path, filename)
        print(f"Processing file: {file_path}")
        
        # Load the CSV file
        df = pd.read_csv(file_path)
        
        # Check if the specific time column exists
        if '/time' not in df.columns:
            print(f"Column '/time' not found in {filename}. Skipping file.")
            continue
        
        # Filter out rows where the time is less than 2 seconds
        df_filtered = df[df['/time'] >= 2].copy()
        
        if not df_filtered.empty:
            print(f"Rows remaining after filtering: {len(df_filtered)}")
            
            # Adjust the time so the first value is set to zero
            initial_time = df_filtered['/time'].iloc[0]  # Get the first time value after filtering
            print(f"Initial time after filtering: {initial_time}")
            df_filtered['/time'] = df_filtered['/time'] - initial_time  # Adjust time to start from zero

            # Save the adjusted DataFrame to a new CSV file
            adjusted_filename = f"{filename}"
            adjusted_file_path = os.path.join(output_folder_path, adjusted_filename)
            df_filtered.to_csv(adjusted_file_path, index=False)

            print(f"Adjusted data saved to {adjusted_file_path}")
        else:
            print(f"No data in {filename} after filtering for time >= 2 seconds. File skipped.")

print("All files processed.")

