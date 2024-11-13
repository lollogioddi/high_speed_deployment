import rosbag
import pandas as pd
import os
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import argparse

def process_bag_file(bag_path, output_directory):
    print(f"Processing: {bag_path}")
    bag = rosbag.Bag(bag_path)
    topics = bag.get_type_and_topic_info()[1].keys()

    data_dict = {}
    timestamps = set()

    for topic in topics:
        for _, msg, t in bag.read_messages(topics=[topic]):
            time_sec = t.to_sec()
            timestamps.add(time_sec)
            if time_sec not in data_dict:
                data_dict[time_sec] = {}
            for field in msg.__slots__:
                data_dict[time_sec][f"{topic}_{field}"] = getattr(msg, field, np.nan)

    # Create a DataFrame with all timestamps and interpolate missing values
    all_times = sorted(list(timestamps))
    df = pd.DataFrame(index=all_times)
    for time in all_times:
        if time in data_dict:
            for key, value in data_dict[time].items():
                df.at[time, key] = value

    # Convert columns to numeric, coercing errors to NaN
    df = df.apply(pd.to_numeric, errors='coerce')

    # Interpolate and fill missing values
    df = df.interpolate(method='index').ffill().bfill()

    # Ensure the output directory exists
    os.makedirs(output_directory, exist_ok=True)
    
    csv_path = os.path.join(output_directory, f"{os.path.basename(bag_path).replace('.bag', '.csv')}")
    df.to_csv(csv_path)
    print(f"Saved to: {csv_path}")

    bag.close()
    return csv_path

def unpack_and_process(args):
    return process_bag_file(*args)

def convert_bag_to_csv(root_directory, output_directory):
    print(f"Looking in root directory: {root_directory}")
    bag_files = []

    for f in os.listdir(root_directory):
        if f.endswith('.bag'):
            bag_files.append((os.path.join(root_directory, f), output_directory))
            print(f"Found bag file: {os.path.join(root_directory, f)}")

    if not bag_files:
        print("No bag files found.")
        return

    with ProcessPoolExecutor() as executor:
        results = list(executor.map(unpack_and_process, bag_files))

    print(f"All files have been processed. Results: {results}")

def main():
    parser = argparse.ArgumentParser(description="Convert ROS bag files to CSV.")
    parser.add_argument("root_directory", type=str, help="The root directory where your BAG files are located.")
    parser.add_argument("output_directory", type=str, help="The directory where CSV files will be saved.")

    args = parser.parse_args()

    convert_bag_to_csv(args.root_directory, args.output_directory)

if __name__ == '__main__':
    main()

