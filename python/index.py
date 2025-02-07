import sys
import os
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

def main():
    # Check if a filename is provided as a command-line argument
    if len(sys.argv) < 2:
        print("Usage: python script.py <filename>")
        sys.exit(1)

    filename = sys.argv[1]

    # Validate if the file exists
    if not os.path.isfile(filename):
        print(f"Error: File '{filename}' does not exist.")
        sys.exit(1)

    # Dictionaries to store data
    x_points = []  # Number of vehicles (numCar)
    y_points = []  # Converted speeds (km/h)
    speed_data = defaultdict(list)  # Store speeds for each numCar

    # Read the file
    with open(filename, 'r', encoding='utf-8') as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line:
                continue

            tokens = [token.strip() for token in line.split(';') if token.strip()]

            if len(tokens) < 2:
                continue

            # Parse numCar
            try:
                num_car = int(tokens[0])  # Number of vehicles
            except ValueError:
                print(f"Error on line {line_number}: Cannot parse numCar '{tokens[0]}'.")
                continue

            expected_token_count = 2 + num_car
            if len(tokens) < expected_token_count:
                print(f"Error on line {line_number}: Expected {expected_token_count} values, found {len(tokens)}.")
                continue

            # Convert speeds from m/s to km/h
            for token in tokens[2:]:
                try:
                    speed_kmh = float(token) * 3.6
                    x_points.append(num_car)  # Use numCar as X-axis value
                    y_points.append(speed_kmh)
                    speed_data[num_car].append(speed_kmh)  # Store for averaging
                except ValueError:
                    print(f"Error on line {line_number}: Cannot convert '{token}' to float.")
                    continue

    if not x_points:
        print("No valid data found. Exiting.")
        sys.exit(1)

    # Plot the data as vertical lines
    plt.figure(figsize=(10, 6))
    plt.vlines(x_points, ymin=[y - 1 for y in y_points], ymax=[y + 1 for y in y_points], color='blue', linewidth=2, alpha=0.3)

    # Plot mean points
    for num_car, speeds in speed_data.items():
        mean_speed = np.mean(speeds)
        plt.scatter(num_car, mean_speed, color='red', marker='o', edgecolors='black', zorder=3)

    plt.xlabel("Number of Vehicles (numCar)")
    plt.ylabel("Average Speed (km/h)")
    plt.title("Number of Vehicles vs Average Speeds")
    
    # No grid
    plt.show()

if __name__ == '__main__':
    main()
