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