import re
import pandas as pd
# Let's read the provided text file and extract the necessary data to form a CSV file
file_path = 'log_file_bs_ue_pos_handover_energy_consumptionMonApr2200:57:302024.txt'

# Reading the file content
with open(file_path, 'r') as file:
    lines = file.readlines()

# Parsing the data from the lines into a DataFrame


data_list = []
for line in lines:
    # Extracting data using regex, assuming format of log is consistent as mentioned
    match = re.search(r"Base Station Power,(\d+\.?\d*),(\d+),(\d+\.?\d*),(\d+\.?\d*)", line)
    if match:
        time_seconds = float(match.group(1))
        b_id = int(match.group(2))
        old_energy_consumption = float(match.group(3))
        new_energy_consumption = float(match.group(4))
        data_list.append([time_seconds, b_id, old_energy_consumption, new_energy_consumption])

# Creating DataFrame
df_csv = pd.DataFrame(data_list, columns=["Time_Seconds", "Base_Station_ID", "Old_Energy_Consumption", "New_Energy_Consumption"])

# Saving DataFrame to CSV file
csv_file_path = 'Converted_Log_File.csv'
df_csv.to_csv(csv_file_path, index=False)

csv_file_path
