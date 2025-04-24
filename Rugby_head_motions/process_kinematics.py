import pandas as pd
import numpy as np
from scipy.integrate import cumulative_trapezoid
import os
from tqdm import tqdm
from multiprocessing import Pool, cpu_count
from scipy.spatial.transform import Rotation

def calculate_position_orientation(df, zero_config=None):
    """
    Calculate position and orientation over time from acceleration and angular velocity data
    
    Parameters:
    df: DataFrame with columns:
        - t(ms): time in milliseconds
        - LinAccX/Y/Z: Linear acceleration in g's
        - RotVelX/Y/Z: Angular velocity in rad/s
    zero_config: dict with boolean flags for zeroing components:
        - zero_x: Zero out X translation
        - zero_y: Zero out Y translation
        - zero_z: Zero out Z translation
        - zero_roll: Zero out Roll rotation
        - zero_pitch: Zero out Pitch rotation
        - zero_yaw: Zero out Yaw rotation
    
    Returns:
    DataFrame with original data plus:
        - X/Y/Z: Position in meters
        - Roll/Pitch/Yaw: Orientation in degrees
    """
    # Set default zero configuration if none provided
    if zero_config is None:
        zero_config = {
            'zero_x': False, 'zero_y': False, 'zero_z': False,
            'zero_roll': False, 'zero_pitch': False, 'zero_yaw': False
        }
    
    # Convert time to seconds
    time_s = df['t(ms)'].values / 1000.0
    dt = np.diff(time_s, prepend=time_s[0])
    
    # Convert accelerations from g's to m/s² and apply zeroing
    acc_x = df['LinAccX'].values * 9.81 if not zero_config['zero_x'] else np.zeros_like(df['LinAccX'].values)
    acc_y = df['LinAccY'].values * 9.81 if not zero_config['zero_y'] else np.zeros_like(df['LinAccY'].values)
    acc_z = df['LinAccZ'].values * 9.81 if not zero_config['zero_z'] else np.zeros_like(df['LinAccZ'].values)
    
    # Get angular velocities and apply zeroing
    omega_x = df['RotVelX'].values if not zero_config['zero_roll'] else np.zeros_like(df['RotVelX'].values)
    omega_y = df['RotVelY'].values if not zero_config['zero_pitch'] else np.zeros_like(df['RotVelY'].values)
    omega_z = df['RotVelZ'].values if not zero_config['zero_yaw'] else np.zeros_like(df['RotVelZ'].values)
    
    # Initialize arrays for position and velocity
    vel_x = cumulative_trapezoid(acc_x, time_s, initial=0)
    vel_y = cumulative_trapezoid(acc_y, time_s, initial=0)
    vel_z = cumulative_trapezoid(acc_z, time_s, initial=0)
    
    pos_x = cumulative_trapezoid(vel_x, time_s, initial=0)
    pos_y = cumulative_trapezoid(vel_y, time_s, initial=0)
    pos_z = cumulative_trapezoid(vel_z, time_s, initial=0)
    
    # Initialize orientation tracking using quaternions
    num_samples = len(time_s)
    orientations = np.zeros((num_samples, 3))  # Store roll, pitch, yaw
    
    # Initial orientation is identity quaternion
    current_quat = Rotation.from_euler('xyz', [0, 0, 0])
    
    # Calculate orientation at each time step
    for i in range(1, num_samples):
        # Get angular velocity at this time step
        omega = [omega_x[i], omega_y[i], omega_z[i]]
        
        # Skip orientation update if all angular velocities are zero
        if np.allclose(omega, [0, 0, 0]):
            orientations[i] = orientations[i-1]
            continue
            
        # Calculate rotation quaternion for this time step
        angle = np.linalg.norm(omega) * dt[i]
        if angle > 0:
            axis = omega / np.linalg.norm(omega)
            delta_rot = Rotation.from_rotvec(axis * angle)
            
            # Update orientation
            current_quat = delta_rot * current_quat
        
        # Convert to Euler angles (in degrees)
        euler_angles = current_quat.as_euler('xyz', degrees=True)
        
        # Explicitly zero out components based on zero_config
        if zero_config['zero_roll']:
            euler_angles[0] = 0
        if zero_config['zero_pitch']:
            euler_angles[1] = 0
        if zero_config['zero_yaw']:
            euler_angles[2] = 0
            
        # Update the quaternion with zeroed angles
        if any([zero_config['zero_roll'], zero_config['zero_pitch'], zero_config['zero_yaw']]):
            current_quat = Rotation.from_euler('xyz', euler_angles, degrees=True)
            
        orientations[i] = euler_angles
    
    # Create results DataFrame
    results_df = pd.DataFrame({
        't(ms)': df['t(ms)'],
        'X(m)': pos_x,
        'Y(m)': pos_y,
        'Z(m)': pos_z,
        'Roll(deg)': orientations[:, 0],
        'Pitch(deg)': orientations[:, 1],
        'Yaw(deg)': orientations[:, 2]
    })
    
    return results_df

def process_file(args):
    """Process a single Excel file to calculate position and orientation"""
    file_path, zero_config, output_dir = args
    try:
        # Read the Excel file
        df = pd.read_excel(file_path)
        
        # Calculate position and orientation with zeroing configuration
        results_df = calculate_position_orientation(df, zero_config)
        
        # Create output filename with zeroed components indicated
        base_name = os.path.splitext(os.path.basename(file_path))[0]
        zeroed = []
        if zero_config['zero_x']: zeroed.append('X')
        if zero_config['zero_y']: zeroed.append('Y')
        if zero_config['zero_z']: zeroed.append('Z')
        if zero_config['zero_roll']: zeroed.append('Roll')
        if zero_config['zero_pitch']: zeroed.append('Pitch')
        if zero_config['zero_yaw']: zeroed.append('Yaw')
        
        suffix = f"_no{'_'.join(zeroed)}" if zeroed else ""
        output_file = os.path.join(output_dir, f"{base_name}{suffix}.xlsx")
        
        # Write results to Position_Orientation sheet
        try:
            with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
                results_df.to_excel(writer, sheet_name='Position_Orientation', index=False)
        except:
            # If file exists, try to append/replace sheet
            with pd.ExcelWriter(output_file, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
                results_df.to_excel(writer, sheet_name='Position_Orientation', index=False)
        
        return True
    except Exception as e:
        print(f"Error processing {file_path}: {str(e)}")
        return False

def process_file_wrapper(args):
    """Wrapper function for process_file to work with multiprocessing"""
    return process_file(args)

def main():
    # Get the current directory using normalized path
    current_dir = os.path.normpath(os.path.dirname(os.path.abspath(__file__)))
    
    # Get all Excel files in the directory
    excel_files = [f for f in os.listdir(current_dir) if f.endswith('.xlsx')]
    file_paths = [os.path.normpath(os.path.join(current_dir, f)) for f in excel_files]
    
    # Get user configuration for zeroing components
    print("\nSelect which motion components to zero out (y/n for each):")
    zero_config = {
        'zero_x': input("Zero out X translation? (y/n): ").lower() == 'y',
        'zero_y': input("Zero out Y translation? (y/n): ").lower() == 'y',
        'zero_z': input("Zero out Z translation? (y/n): ").lower() == 'y',
        'zero_roll': input("Zero out Roll rotation? (y/n): ").lower() == 'y',
        'zero_pitch': input("Zero out Pitch rotation? (y/n): ").lower() == 'y',
        'zero_yaw': input("Zero out Yaw rotation? (y/n): ").lower() == 'y'
    }
    
    # Create output directory name based on zeroed components
    zeroed = []
    if zero_config['zero_x']: zeroed.append('X')
    if zero_config['zero_y']: zeroed.append('Y')
    if zero_config['zero_z']: zeroed.append('Z')
    if zero_config['zero_roll']: zeroed.append('Roll')
    if zero_config['zero_pitch']: zeroed.append('Pitch')
    if zero_config['zero_yaw']: zeroed.append('Yaw')
    
    folder_suffix = f"_no{'_'.join(zeroed)}" if zeroed else "_processed"
    output_dir = os.path.join(current_dir, f"processed_data{folder_suffix}")
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Create process arguments with configuration and output directory
    process_args = [(f, zero_config, output_dir) for f in file_paths]
    
    # Create a pool of workers
    num_processes = min(cpu_count(), 8)  # Use up to 8 cores
    
    print(f"\nProcessing {len(file_paths)} files using {num_processes} processes...")
    print("Components zeroed:", 
          ', '.join([k[5:].upper() for k, v in zero_config.items() if v]) or "None")
    print(f"Output directory: {output_dir}")
    
    # Process files in parallel with progress bar
    with Pool(num_processes) as pool:
        results = list(tqdm(pool.imap(process_file_wrapper, process_args), 
                          total=len(file_paths)))
    
    # Print summary
    successful = sum(results)
    print(f"\nProcessing complete!")
    print(f"Successfully processed: {successful} files")
    print(f"Failed: {len(file_paths) - successful} files")
    print(f"Files saved in: {output_dir}")

if __name__ == '__main__':
    main()