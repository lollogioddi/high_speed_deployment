import pandas as pd
from tqdm import tqdm
import os

def process_csv(file_path):
    data = pd.read_csv(file_path, dtype={17: 'str', 18: 'str'})
    
    # Forward-fill the timestamps to ensure each row has a timestamp
    data['/time_in_sec_data'] = data['/time_in_sec_data'].ffill()
    
    # Initialize a list to collect the combined rows
    combined_rows = []
    
    # Get unique timestamps
    unique_timestamps = data['/time_in_sec_data'].unique()
    
    # Process each unique timestamp with a progress bar
    for timestamp in unique_timestamps:
        # Find all rows with the same timestamp
        same_timestamp_rows = data[data['/time_in_sec_data'] == timestamp]
        
        if not same_timestamp_rows.empty:
            # Find the index of the first and last occurrence of this timestamp
            first_idx = same_timestamp_rows.index[0]
            last_idx = same_timestamp_rows.index[-1]
            
            # Get all rows between the first and last occurrence
            rows_to_combine = data.loc[first_idx:last_idx]
            
            # Combine rows by filling missing values from all rows in the range
            combined_row = rows_to_combine.iloc[0].copy()
            for col in rows_to_combine.columns:
                if col != '/time':  # Skip the timestamp column
                    non_na_values = rows_to_combine[col].dropna().values
                    if len(non_na_values) > 0:
                        combined_row[col] = non_na_values[0]
                    else:
                        combined_row[col] = None
            
            combined_rows.append(combined_row)
    
    # Create a DataFrame from the combined rows
    consolidated_data = pd.DataFrame(combined_rows)
    
    # Write the consolidated data to the same CSV file
    consolidated_data.to_csv(file_path, index=False)

# Define the main directory path
main_directory = '/home/lollogioddi/Documents/thesis/stesura/raccolta_dati_finale_2/Real_12sett/adaptive/presentation/csv'

# Traverse the directory structure
for root, dirs, files in os.walk(main_directory):
    for file in files:
        if file.endswith('.csv'):
            file_path = os.path.join(root, file)
            tqdm.write(f"Processing {file_path}")
            process_csv(file_path)

print("All files have been processed and fixed.")

