# Platform Controller

This program calculates the required slider positions and motor angles for a 3-slider platform based on desired platform position and orientation data from an Excel file.

## Features

- Reads trajectory data from Excel file (from sheet named 'Position_Orientation')
- Optimizes platform orientation to minimize slider movements
- Calculates required slider positions and motor angles
- Computes and reports peak velocities and accelerations
- Validates motion constraints and range of motion
- Provides visualization of results

## Installation

1. Create a virtual environment (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

## Input File Format

Create an Excel file with a sheet named 'Position_Orientation' containing the following columns:
- `t(ms)`: Time in milliseconds
- `X(m)`: X-position in meters
- `Y(m)`: Y-position in meters
- `Z(m)`: Z-position in meters
- `Roll(deg)`: Roll angle in degrees
- `Pitch(deg)`: Pitch angle in degrees
- `Yaw(deg)`: Yaw angle in degrees

Note: The program automatically converts time from milliseconds to seconds internally.

## Usage

1. Prepare your Excel file with the required 'Position_Orientation' sheet and columns as described above.

2. Run the program:
   ```bash
   python platform_controller.py
   ```

3. When prompted:
   - Enter the path to your Excel file
   - Enter the leg length (or press Enter for default 0.5m)
   - Enter the maximum rail travel (or press Enter for default 0.5m)

4. The program will:
   - Load and validate the trajectory data
   - Convert time values from milliseconds to seconds
   - Calculate optimal slider positions and motor angles
   - Display peak velocities and accelerations
   - Show plots of slider positions, velocities, and accelerations

## Output

The program generates:
1. Console output showing:
   - Data validation results
   - Peak velocities and accelerations
2. Three plots showing:
   - Slider positions over time (in seconds)
   - Slider velocities over time
   - Slider accelerations over time

## Error Handling

The program will:
- Validate the Excel file path and sheet name
- Check for required columns with exact header names
- Validate that positions are reachable
- Check that slider positions are within range
- Optimize platform orientation to minimize slider movements
- Report warnings for any unreachable positions

## Detailed Explanation of `platform_controllerMP.py`

The `platform_controllerMP.py` script is designed to control and optimize the motion of a parallel robotic platform, such as a Stewart platform. Below is a detailed explanation of its components and functionality:

### Key Features
- **Trajectory Processing**: Reads trajectory data from an Excel file and calculates slider positions, motor angles, and joint angles for the platform.
- **Optimization**: Optimizes platform position and rotation offsets to minimize slider movements and ensure feasibility.
- **Visualization**: Provides plots of slider positions, velocities, and accelerations.
- **Export**: Saves processed trajectory data to an Excel file with detailed statistics.

### Key Classes and Functions

#### `ThreadSafeCounter`
- A utility class to safely increment a counter in a multi-threaded environment.

#### `chunk_data`
- Splits a `pandas.DataFrame` into smaller chunks for parallel processing.

#### `optimize_with_initial_guess`
- Performs optimization for a single initial guess using the `scipy.optimize.minimize` function.

#### `PlatformController`
- The core class that encapsulates the logic for controlling and optimizing the platform's motion.

##### Initialization (`__init__`)
- Initializes the platform's geometric parameters, including leg length, rail travel, and attachment points.

##### Key Methods
1. **`transform_platform_points`**
   - Transforms platform attachment points from local to world coordinates using position and rotation.

2. **`calculate_joint_angles`**
   - Calculates joint angles for the platform's legs and sliders based on the platform's position and orientation.

3. **`calculate_slider_positions`**
   - Computes slider positions and motor angles required to achieve a given platform position and orientation.

4. **`optimize_platform_orientation`**
   - Optimizes the platform's orientation to minimize slider movements while maintaining a target position and orientation.

5. **`optimize_offsets`**
   - Optimizes position and rotation offsets to improve motion feasibility and efficiency.

6. **`objective_function`**
   - Defines the objective function for optimization, penalizing infeasible solutions and rewarding efficient motion.

7. **`process_trajectory`**
   - Processes a trajectory to compute slider positions, motor angles, and joint angles, with optional optimization.

8. **`export_results_to_excel`**
   - Exports the processed trajectory data to an Excel file with multiple sheets for positions, velocities, accelerations, and statistics.

9. **`plot_results`**
   - Visualizes the trajectory results, including slider positions, velocities, and accelerations.

### Workflow
1. **Input Trajectory Data**:
   - Reads trajectory data from an Excel file.
   - Validates and converts time values from milliseconds to seconds.

2. **Trajectory Processing**:
   - Calculates slider positions and motor angles for each point in the trajectory.
   - Checks for feasibility (e.g., whether the leg lengths and slider positions are within bounds).

3. **Optimization**:
   - Optimizes position and rotation offsets to minimize slider movements and ensure feasibility.

4. **Output**:
   - Exports the processed trajectory data to an Excel file.
   - Generates plots to visualize the platform's motion.

### Applications
This script is designed for controlling and optimizing the motion of parallel robotic platforms, which are used in:
- **Flight simulators**: To simulate aircraft motion.
- **Surgical robots**: For precise positioning in medical procedures.
- **Industrial automation**: For tasks requiring high precision and flexibility.
- **Virtual reality platforms**: To provide motion feedback.

### Example Use Case
- A user has a trajectory for a platform and wants to ensure that the platform can follow the trajectory without exceeding the physical limits of its actuators.
- The script processes the trajectory, identifies infeasible points, and optimizes the platform's offsets to improve feasibility and efficiency.