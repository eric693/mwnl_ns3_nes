
# Correct Time Integer 
import re
import csv
from collections import defaultdict

# Read the provided log file and convert it to a CSV format with node IDs as columns and time as rows.

def parse_position_data_to_csv(log_file_path, csv_file_path):
    # This will hold the mapping from time and node ID to position.
    data = defaultdict(dict)
    node_ids = set()
    times = set()
    
    # Define the regular expression to match each line of the log file.
    log_pattern = re.compile(r'(\d+(?:\.\d+)?|\d+\.\d+e[+-]\d+) Node (\d+): Position: (\d+\.\d+), (\d+\.\d+), (\d+\.\d+)')
    # log_pattern = re.compile(r'(\d+(?:\.\d+)?|\d+\.\d+e[+-]\d+) Node (\d+): Position: (\d+), (\d+), (\d+)')
    # Open the log file and parse its contents.
    with open(log_file_path, 'r') as log_file:
        for line in log_file:
            match = log_pattern.match(line)
            if match:
                # Extract time, node ID, and position information.
                time, node_id, x, y, z = match.groups()
                data[float(time)][int(node_id)] = (float(x), float(y), float(z))
                node_ids.add(int(node_id))
                times.add(float(time))

    # Convert the times and node IDs to sorted lists for consistent ordering.
    sorted_times = sorted(times)
    sorted_node_ids = sorted(node_ids)
    
    # Open the CSV file for writing.
    with open(csv_file_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        
        # Write the header row with node IDs.
        writer.writerow(['Time (s)'] + [f'Node {node_id}' for node_id in sorted_node_ids])
        
        # Write the position data rows, one for each time point.
        for time in sorted_times:
            row = [time]
            for node_id in sorted_node_ids:
                # Write an empty string if no position data is available for a given node at the current time.
                position = data[time].get(node_id, ("", "", ""))
                # Format position as a single string "x, y, z"
                row.append(', '.join(map(str, position)))
            writer.writerow(row)


if __name__ == "__main__":
    input_file_path = 'positionLog.txt'  # 输入文件路径
    output_file_path = 'ue_pos_preprocess.csv'  # 输出CSV文件的路径
    parse_position_data_to_csv(input_file_path, output_file_path)


