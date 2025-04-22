import pandas as pd
import numpy as np
import os

def calculate_peak_values(acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, pos_x, pos_y, pos_z,
                        ang_vel_x, ang_vel_y, ang_vel_z, rot_x, rot_y, rot_z):
    # Calculate peak values (both max and min)
    peaks = {
        'Linear Acceleration (m/s²)': {
            'X_Max': np.max(acc_x),
            'X_Min': np.min(acc_x),
            'Y_Max': np.max(acc_y),
            'Y_Min': np.min(acc_y),
            'Z_Max': np.max(acc_z),
            'Z_Min': np.min(acc_z)
        },
        'Linear Velocity (m/s)': {
            'X_Max': np.max(vel_x),
            'X_Min': np.min(vel_x),
            'Y_Max': np.max(vel_y),
            'Y_Min': np.min(vel_y),
            'Z_Max': np.max(vel_z),
            'Z_Min': np.min(vel_z)
        },
        'Linear Displacement (m)': {
            'X_Max': np.max(pos_x),
            'X_Min': np.min(pos_x),
            'Y_Max': np.max(pos_y),
            'Y_Min': np.min(pos_y),
            'Z_Max': np.max(pos_z),
            'Z_Min': np.min(pos_z)
        },
        'Angular Velocity (deg/s)': {
            'X_Max': np.max(ang_vel_x),
            'X_Min': np.min(ang_vel_x),
            'Y_Max': np.max(ang_vel_y),
            'Y_Min': np.min(ang_vel_y),
            'Z_Max': np.max(ang_vel_z),
            'Z_Min': np.min(ang_vel_z)
        },
        'Angular Displacement (deg)': {
            'X_Max': np.max(rot_x),
            'X_Min': np.min(rot_x),
            'Y_Max': np.max(rot_y),
            'Y_Min': np.min(rot_y),
            'Z_Max': np.max(rot_z),
            'Z_Min': np.min(rot_z)
        }
    }
    
    # Convert to DataFrame
    peak_df = pd.DataFrame.from_dict(peaks, orient='index')
    
    # Round values to 3 decimal places for better readability
    peak_df = peak_df.round(3)
    
    return peak_df

def process_imu_data(file_path, sampling_rate=1000):  # Assuming 1000Hz sampling rate
    # Read the Excel file
    df = pd.read_excel(file_path)
    
    # Print column names for debugging
    print("Available columns in the Excel file:")
    print(df.columns.tolist())
    
    # Get time vector from the Time column
    time = df['Time'].values
    dt = 1.0 / sampling_rate
    
    # Extract accelerations and angular velocities
    acc_x = df['AccelX'].values
    acc_y = df['AccelY'].values
    acc_z = df['AccelZ'].values
    
    ang_vel_x = df['RotVelX'].values
    ang_vel_y = df['RotVelY'].values
    ang_vel_z = df['RotVelZ'].values
    
    
    # Convert angular velocities to degrees/second
    ang_vel_x = np.degrees(ang_vel_x)
    ang_vel_y = np.degrees(ang_vel_y)
    ang_vel_z = np.degrees(ang_vel_z)
    
    # Integrate accelerations to get velocities using numpy's cumsum
    vel_x = np.cumsum(acc_x) * dt
    vel_y = np.cumsum(acc_y) * dt
    vel_z = np.cumsum(acc_z) * dt
    
    # Integrate velocities to get positions
    pos_x = np.cumsum(vel_x) * dt
    pos_y = np.cumsum(vel_y) * dt
    pos_z = np.cumsum(vel_z) * dt
    
    # Integrate angular velocities to get rotation angles
    rot_x = np.cumsum(ang_vel_x) * dt
    rot_y = np.cumsum(ang_vel_y) * dt
    rot_z = np.cumsum(ang_vel_z) * dt
    
    # Calculate peak values
    peak_values_df = calculate_peak_values(
        acc_x, acc_y, acc_z,
        vel_x, vel_y, vel_z,
        pos_x, pos_y, pos_z,
        ang_vel_x, ang_vel_y, ang_vel_z,
        rot_x, rot_y, rot_z
    )
    
    # Create output dataframe
    output_df = pd.DataFrame({
        'Time': time,
        'Position_X': pos_x,
        'Position_Y': pos_y,
        'Position_Z': pos_z,
        'Rotation_X': rot_x,
        'Rotation_Y': rot_y,
        'Rotation_Z': rot_z
    })
    
    return output_df, peak_values_df

def main():
    try:
        # Input file path
        input_file = os.path.join('RugbyIMUData1', 'Rugby_1221_for_blender.xlsx')
        
        # Check if file exists
        if not os.path.exists(input_file):
            print(f"Error: Input file not found at {input_file}")
            return
            
        # Output file path
        output_file = 'Rugby_1221_processed_positions_rotations.xlsx'
        
        # Process the data
        result_df, peak_values_df = process_imu_data(input_file)
        
        # Save to Excel with multiple sheets
        with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
            result_df.to_excel(writer, sheet_name='Motion Data', index=False)
            peak_values_df.to_excel(writer, sheet_name='Peak Values')
            
        print(f"Processed data saved to {output_file}")
        
    except Exception as e:
        print(f"An error occurred: {str(e)}")

if __name__ == "__main__":
    main()
