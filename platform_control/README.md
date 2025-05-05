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
   - Enter the leg length (or press Enter for default 0.3m)
   - Enter the maximum rail travel (or press Enter for default 0.5m)
   - Choose whether to log optimization attempts (y/N default No)

4. The program will:
   - Load and validate the trajectory data
   - Convert time values from milliseconds to seconds
   - Process trajectory without optimization (for comparison)
   - Optimize platform position and rotation offsets
   - Calculate optimal slider positions and motor angles
   - Display peak velocities and accelerations
   - Show improvement percentages compared to non-optimized motion
   - Show plots of slider positions, velocities, and accelerations

## Output

The program generates:
1. Console output showing:
   - Non-optimized results (initial position, orientation, peak velocities, and accelerations)
   - Optimization progress and results
   - Best feasible solution found
   - Improvement percentages for velocity and acceleration reduction
   - Peak velocities and accelerations
2. Excel file containing:
   - Positions sheet: Time-series data of slider positions and platform motion
   - Velocities sheet: Calculated velocities for each slider
   - Accelerations sheet: Calculated accelerations for each slider
   - Statistics sheet: Comprehensive metrics including ranges and optimization results
3. Debug log file (if logging enabled) containing:
   - Detailed optimization attempts
   - Score for each attempt
   - Position and rotation offsets tried
   - Peak velocities and accelerations
   - Final optimization results summary
4. Three plots showing:
   - Slider positions over time (in seconds)
   - Slider velocities over time
   - Slider accelerations over time

## New Features and Improvements

### Optimization Enhancements
- Added option to enable/disable optimization attempt logging
- Improved acceleration constraints (50 m/s² maximum)
- Enhanced penalty system for exceeding velocity and acceleration limits
- Better handling of infeasible solutions
- Improved tracking of best feasible solution

### File Handling
- Better error handling for Excel file access
- Clear message when Excel file is in use
- Automatic debug log file creation and management
- Structured output files with detailed statistics

### Performance Optimization
- Optional logging to reduce disk I/O during optimization
- Improved parallel processing of optimization attempts
- Better memory management for large trajectories

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

## Batch Processing and Size Optimization from `Batch_process_motions_size_optimise.py`

The `Batch_process_motions_size_optimise.py` script provides automated processing of multiple motion files while optimizing the platform's physical dimensions (leg length and rail travel) for best performance.

### Features
- Batch processes multiple Excel files in parallel
- Two-phase optimization strategy for leg lengths:
  - Phase 1: Coarse search (0.2m to 0.75m in 0.1m steps)
  - Phase 2: Fine-tuning (±0.1m around best coarse result in 0.05m steps)
- Tests multiple rail lengths (0.5m, 0.6m, 0.75m)
- Zeroing configuration options for each motion component
- Comprehensive optimization metrics:
  - Peak velocities and accelerations
  - Movement efficiency score
  - Improvement percentages
- Detailed results export with statistics

### Usage

1. Run the script:
   ```bash
   python Batch_process_motions_size_optimise.py
   ```

2. When prompted:
   - Enter the directory path containing Excel files (or press Enter for current directory)
   - Choose which motion components to zero out (X, Y, Z translations and Roll, Pitch, Yaw rotations)

3. The script will:
   - Process each file in parallel
   - Test multiple leg lengths and rail lengths
   - Find the optimal configuration for each motion
   - Generate a summary Excel file with results

### Output

The script generates a comprehensive Excel file named "leg_length_optimization_results.xlsx" containing:

1. All Results sheet:
   - File names
   - Optimal rail and leg lengths
   - Peak velocities and accelerations
   - Improvement percentages
   - Overall scores

2. Statistics sheet:
   - Average leg length
   - Standard deviation
   - Most common leg length
   - Average velocity improvement
   - Success rate

3. Length Distribution sheet:
   - Distribution of optimal leg lengths
   - Frequency counts

### Optimization Strategy

The script uses a two-phase optimization approach:

1. Coarse Phase:
   - Tests leg lengths from 0.2m to 0.75m in 0.1m increments
   - Identifies the most promising length range

2. Fine-tuning Phase:
   - Explores ±0.1m around the best coarse result
   - Uses smaller 0.05m increments
   - Finds the optimal configuration

The optimization score combines:
- Peak velocities
- Peak accelerations
- Movement efficiency
- Feasibility constraints

### Applications

This script is particularly useful for:
- Finding optimal platform dimensions for a set of motions
- Batch processing multiple motion files
- Comparing performance across different configurations
- Generating statistical insights about optimal platform sizes

## Batch Processing with `Batch_process_motions.py`

The `Batch_process_motions.py` script provides automated processing of multiple motion files in parallel, with built-in trajectory optimization.

### Features
- Parallel processing of multiple Excel files using multiprocessing
- Automatic file discovery in specified directory
- Configurable zeroing options for each motion component
- Two-phase processing strategy:
  1. Initial processing without optimization (for baseline metrics)
  2. Optimized processing with position and rotation offset optimization
- Progress tracking with tqdm progress bar
- Comprehensive error handling and reporting
- Performance comparison between optimized and non-optimized results

### Usage

1. Run the script:
   ```bash
   python Batch_process_motions.py
   ```

2. When prompted:
   - Enter the directory path containing Excel files (or press Enter for current directory)
   - Configure zeroing options for each motion component (X, Y, Z translations and Roll, Pitch, Yaw rotations)
   - Specify platform parameters:
     - Leg length (default 0.3m)
     - Maximum rail travel (default 0.5m)

3. The script will:
   - Scan the directory for Excel files
   - Process each file in parallel using multiple CPU cores
   - Display progress with a progress bar
   - Show optimization improvements for each file
   - Provide a summary of successful and failed operations

### Processing Steps

For each file, the script:

1. **Data Loading and Initial Processing**:
   - Reads the Excel file
   - Calculates initial position and orientation using zeroing configuration
   - Saves results to 'Position_Orientation' sheet

2. **Non-optimized Processing**:
   - Processes trajectory without optimization
   - Calculates and displays initial metrics:
     - Peak velocities
     - Peak accelerations
     - Initial platform configuration

3. **Optimization Phase**:
   - Optimizes platform position and rotation offsets
   - Applies optimized offsets
   - Processes trajectory with optimization
   - Calculates improvement metrics:
     - Velocity reduction percentage
     - Acceleration reduction percentage

### Output

For each processed file:

1. Console output showing:
   - Initial platform configuration
   - Non-optimized performance metrics
   - Optimization results
   - Improvement percentages

2. Updated Excel file containing:
   - Position_Orientation sheet with processed data
   - Additional sheets with optimization results

3. Summary statistics:
   - Total number of files processed
   - Success/failure counts
   - Error details for failed files

### Performance Features

- Utilizes half of available CPU cores to avoid system overload
- Parallel processing with ProcessPoolExecutor
- Progress tracking with tqdm
- Efficient memory management
- Comprehensive error handling and reporting

### Applications

This script is ideal for:
- Processing large batches of motion capture data
- Analyzing multiple platform trajectories
- Comparing optimized vs non-optimized performance
- Validating motion feasibility across multiple scenarios

### Error Handling

The script provides:
- Robust directory and file validation
- Per-file error catching and reporting
- Clear success/failure statistics
- Detailed error messages for troubleshooting