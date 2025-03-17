import sys
import os
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

# =======================
# User-Configurable Parameters
# =======================

# Parameters for the vertical bars representing individual data points
BAR_COLOR = 'blue'             # Color of the vertical bars
BAR_WIDTH = 1                  # Width of the bars (in data units)
BAR_VERTICAL_OFFSET = 0.3      # Vertical offset: each bar spans from (y - offset) to (y + offset)
BAR_ALPHA = 0.1                # Opacity of the bars (0.0 to 1.0)

# Parameters for the mean speed trend line
MEAN_LINE_COLOR = 'red'        # Color of the mean speed line
MEAN_LINE_STYLE = '-'          # Style of the mean speed line
MEAN_LINE_WIDTH = 2            # Thickness of the mean speed line

# Parameters for the trend line (interpolation)
TREND_LINE_COLOR = 'green'     # Color of the trend line
TREND_LINE_STYLE = '-'         # Style of the trend line
TREND_LINE_WIDTH = 2           # Thickness of the trend line
TREND_DEGREE = 1               # Degree of the polynomial for the trend line (1 = linear)

# Parameters for the standard deviation bands
STD_LINE_COLOR = 'purple'      # Color of the standard deviation lines
STD_LINE_STYLE = '--'          # Style of the standard deviation lines
STD_LINE_WIDTH = 1.5           # Thickness of the standard deviation lines

# Parameters for the x-axis labels
X_LABEL_STEP = 4

# =======================
# Main Code
# =======================

def main():
    # Check if a filename is provided as a command-line argument
    if len(sys.argv) < 2:
        print("Usage: python script.py <filename>")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    # Validate that the file exists
    if not os.path.isfile(filename):
        print(f"Error: File '{filename}' does not exist.")
        sys.exit(1)
    
    # Lists to store data points
    x_points = []              # Number of vehicles (numCar)
    y_points = []              # Converted speeds (km/h)
    speed_data = defaultdict(list)  # Dictionary mapping numCar to a list of speeds
    density_mapping = {}        # Store density values for each numCar
    
    # Read the file
    with open(filename, 'r', encoding='utf-8') as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line:
                continue
            
            # Split the line using ';' as the delimiter and remove empty tokens
            tokens = [token.strip() for token in line.split(';') if token.strip()]
            
            if len(tokens) < 2:
                continue
            
            # Parse numCar (the number of vehicles) and density
            try:
                num_car = int(tokens[0])
                density = float(tokens[1])  # Store density for the x-axis label
                density_mapping[num_car] = density  # Ensure each numCar has a unique density mapping
            except ValueError:
                print(f"Error on line {line_number}: Cannot parse numCar or density '{tokens[:2]}'.")
                continue
            
            expected_token_count = 2 + num_car
            if len(tokens) < expected_token_count:
                print(f"Error on line {line_number}: Expected {expected_token_count} values, found {len(tokens)}.")
                continue
            
            # Process each speed value (tokens from index 2 onward), converting from m/s to km/h
            for token in tokens[2:]:
                try:
                    speed_kmh = float(token) * 3.6
                    x_points.append(num_car)           # Use numCar as the x-value
                    y_points.append(speed_kmh)         # Store the speed (km/h)
                    speed_data[num_car].append(speed_kmh)  # Group speeds by numCar for averaging
                except ValueError:
                    print(f"Error on line {line_number}: Cannot convert '{token}' to float.")
                    continue
    
    if not x_points:
        print("No valid data found. Exiting.")
        sys.exit(1)
    
    # Compute the mean speed and standard deviation for each unique numCar
    unique_x = sorted(speed_data.keys())
    mean_y = [np.mean(speed_data[num]) for num in unique_x]
    std_y = [np.std(speed_data[num]) for num in unique_x]  # Compute standard deviation

    # Compute upper and lower bounds (±1 standard deviation)
    upper_y = [mean + std for mean, std in zip(mean_y, std_y)]
    lower_y = [mean - std for mean, std in zip(mean_y, std_y)]
    
    # Fit a linear trend line (degree 1)
    trend_poly = np.polyfit(unique_x, mean_y, TREND_DEGREE)
    trend_func = np.poly1d(trend_poly)
    
    # Generate smooth x values for plotting the trend curve
    x_smooth = np.linspace(min(unique_x), max(unique_x), 300)
    y_smooth = trend_func(x_smooth)
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot the individual data points as vertical bars using plt.bar
    bottoms = [y - BAR_VERTICAL_OFFSET for y in y_points]
    heights = [2 * BAR_VERTICAL_OFFSET for _ in y_points]
    ax.bar(x_points, heights, width=BAR_WIDTH, bottom=bottoms, color=BAR_COLOR,
           alpha=BAR_ALPHA, align='center')
    
    # Plot the mean speed as a continuous red line
    ax.plot(unique_x, mean_y, color=MEAN_LINE_COLOR, linestyle=MEAN_LINE_STYLE,
            linewidth=MEAN_LINE_WIDTH, label="Mean Speed")
    
    # Plot the trend (interpolation) curve
    ax.plot(x_smooth, y_smooth, color=TREND_LINE_COLOR, linestyle=TREND_LINE_STYLE,
            linewidth=TREND_LINE_WIDTH, label="Trend Curve")
    
    # Plot ±1 standard deviation lines
    ax.plot(unique_x, upper_y, color=STD_LINE_COLOR, linestyle=STD_LINE_STYLE,
            linewidth=STD_LINE_WIDTH, label="+1 Std Dev")
    ax.plot(unique_x, lower_y, color=STD_LINE_COLOR, linestyle=STD_LINE_STYLE,
            linewidth=STD_LINE_WIDTH, label="-1 Std Dev")

    # Set x-axis labels with "numCar (density)"
    x_labels = [f"{num} ({density_mapping[num]:.0f})" if i % X_LABEL_STEP == 0 else "" 
            for i, num in enumerate(unique_x)]
    ax.set_xticks(unique_x)
    ax.set_xticklabels(x_labels, rotation=45, ha='right')  # Rotate for better readability
    ax.set_xlim(min(unique_x)-0.5, max(unique_x)+0.5)
    
    ax.set_xlabel("Number of Vehicles (Density)")
    ax.set_ylabel("Average Speed (km/h)")
    ax.set_title("Number of Vehicles vs Average Speeds with Std Deviation")
    ax.legend()
    
    # Display the plot (grid is not added)
    plt.show()

if __name__ == '__main__':
    main()
