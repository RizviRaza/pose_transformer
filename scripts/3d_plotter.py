import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Step 1: Read Excel File
def read_excel(file_path, sheet_name='Final'):
    # Read the specified sheet from the Excel file
    data = pd.read_excel(file_path, sheet_name=sheet_name)
    return data

# Step 2: Extract coordinates (Goal and Drone)
def extract_coordinates(data):
    # Convert columns to numeric, coercing errors (non-numeric entries become NaN)
    data.iloc[:, 2:5] = data.iloc[:, 2:5].apply(pd.to_numeric, errors='coerce')

    # Drop rows where any of the x, y, z values are NaN
    data = data.dropna(subset=data.columns[2:5])

    goal_coords = data.iloc[::2, 2:5].to_numpy()  # Extract rows for Goal (every other row starting from 0)
    drone_coords = data.iloc[1::2, 2:5].to_numpy()  # Extract rows for Drone (every other row starting from 1)
    return goal_coords, drone_coords

# Step 3: Plot 3D coordinates
def plot_3d(goal_coords, drone_coords, goal_color='blue', drone_color='red', 
            goal_label='Goal Positions', drone_label='Drone Positions', title='3D Plot of Coordinates'):
    # Extract x, y, z for Goal and Drone
    goal_x, goal_y, goal_z = goal_coords[:, 0], goal_coords[:, 1], goal_coords[:, 2]
    drone_x, drone_y, drone_z = drone_coords[:, 0], drone_coords[:, 1], drone_coords[:, 2]

    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot Goal positions
    ax.scatter(goal_x, goal_y, goal_z, color=goal_color, label=goal_label, s=50)

    # Plot Drone positions
    ax.scatter(drone_x, drone_y, drone_z, color=drone_color, label=drone_label, s=50)

    # Connect subsequent Goal positions with lines
    ax.plot(goal_x, goal_y, goal_z, color=goal_color, linestyle='-', label=f'{goal_label} Path')

    # Connect subsequent Drone positions with lines
    ax.plot(drone_x, drone_y, drone_z, color=drone_color, linestyle='-', label=f'{drone_label} Path')

    # Adjust the zoom out (expand limits)
    padding = 0.35  # Adjust padding as needed
    x_min, x_max = min(min(goal_x), min(drone_x)), max(max(goal_x), max(drone_x))
    y_min, y_max = min(min(goal_y), min(drone_y)), max(max(goal_y), max(drone_y))
    z_min, z_max = min(min(goal_z), min(drone_z)), max(max(goal_z), max(drone_z))

    ax.set_xlim([x_min - padding, x_max + padding])
    ax.set_ylim([y_min - padding, y_max + padding])
    ax.set_zlim([z_min - padding, z_max + padding])

    # Label the axes
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    # Add legend
    ax.legend()

    # Add title
    ax.set_title(title)

    # Show the plot
    plt.show()

# Usage Example
if __name__ == "__main__":
    # File path to your Excel file
    file_path = '/home/raza/Downloads/Experiments_data.xlsx'

    # Read the data from the specified sheet
    data = read_excel(file_path, sheet_name='Final')

    # Extract coordinates
    goal_coords, drone_coords = extract_coordinates(data)

    # Plot the 3D coordinates
    plot_3d(goal_coords, drone_coords, 
            goal_color='blue', 
            drone_color='red', 
            goal_label='Goal', 
            drone_label='Drone', 
            title='3D Plot of Goal and Drone Positions')
