import sys
import os
import pandas as pd
import logging
from concurrent.futures import ProcessPoolExecutor

def setup_logging():
    logging.basicConfig(level=logging.ERROR, filename='error.log', filemode='w',
                        format='%(asctime)s - %(levelname)s - %(message)s')

def process_file(full_path, timestamp_column):
    try:
        df = pd.read_csv(full_path)
        df_unique = df[~df.duplicated(subset=timestamp_column, keep='first')]
        df_unique.to_csv(full_path, index=False)
        print(f'Successfully processed file: {full_path}')
    except Exception as e:
        logging.error(f'Error processing file {full_path}: {e}')

def main(directory, timestamp_column):
    if not os.path.exists(directory):
        logging.error(f"The specified directory does not exist: {directory}")
        return

    all_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.csv'):
                all_files.append(os.path.join(root, file))

    with ProcessPoolExecutor() as executor:
        futures = [executor.submit(process_file, f, timestamp_column) for f in all_files]
        for future in futures:
            try:
                future.result()
            except Exception as e:
                logging.error(f"Error during processing: {e}")

if __name__ == "__main__":
    setup_logging()
    if len(sys.argv) < 2:
        print("Usage: python script.py <directory>")
        sys.exit(1)

    directory = sys.argv[1]
    timestamp_column = '/time_in_sec_data'
    main(directory, timestamp_column)
