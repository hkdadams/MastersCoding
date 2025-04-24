import pandas as pd
import os
from multiprocessing import Pool, cpu_count
from tqdm import tqdm
import numpy as np
from scipy.integrate import cumulative_trapezoid

def get_peak_with_sign(series):
    # Find the value with the maximum absolute magnitude but preserve its sign
    idx = series.abs().idxmax()
    return series.iloc[idx]

def calculate_displacement(acceleration, time_ms):
    """
    Calculate displacement by double integration of acceleration
    acceleration: acceleration in g's
    time_ms: time in milliseconds
    Returns: maximum displacement in meters
    """
    # Convert time to seconds and acceleration to m/s²
    time_s = time_ms / 1000.0
    acc_ms2 = acceleration * 9.81
    
    # First integration to get velocity
    velocity = cumulative_trapezoid(acc_ms2, time_s, initial=0)
    
    # Second integration to get displacement
    displacement = cumulative_trapezoid(velocity, time_s, initial=0)
    
    # Return maximum absolute displacement
    return get_peak_with_sign(pd.Series(displacement))

def calculate_angle(angular_velocity, time_ms):
    """
    Calculate angle by integrating angular velocity
    angular_velocity: angular velocity in rad/s
    time_ms: time in milliseconds
    Returns: maximum angle in degrees
    """
    # Convert time to seconds
    time_s = time_ms / 1000.0
    
    # Integrate angular velocity to get angle
    angle_rad = cumulative_trapezoid(angular_velocity, time_s, initial=0)
    
    # Convert to degrees and return maximum
    angle_deg = np.degrees(angle_rad)
    return get_peak_with_sign(pd.Series(angle_deg))

def calculate_hic(acc_resultant, time_ms, window_size_ms):
    """
    Calculate HIC (Head Injury Criterion) for a given time window
    acc_resultant: resultant linear acceleration in g's
    time_ms: time in milliseconds
    window_size_ms: size of sliding window in milliseconds
    Returns: (HIC value, start_time, end_time)
    """
    # Convert time to seconds for calculation
    time_s = time_ms / 1000.0
    window_size_s = window_size_ms / 1000.0
    
    max_hic = 0
    max_start = 0
    max_end = 0
    
    for i in range(len(time_s)):
        for j in range(i + 1, len(time_s)):
            # Check if window size is exceeded
            if time_s[j] - time_s[i] > window_size_s:
                break
                
            # Calculate HIC for this window
            dt = time_s[j] - time_s[i]
            if dt > 0:  # Avoid division by zero
                # Calculate average acceleration over the interval (already in g's)
                avg_acc = np.mean(acc_resultant[i:j+1])
                hic = dt * (avg_acc**2.5)  # Data already in g's
                
                if hic > max_hic:
                    max_hic = hic
                    max_start = time_ms[i]
                    max_end = time_ms[j]
    
    return max_hic, max_start, max_end

def calculate_bric(rot_vel_x, rot_vel_y, rot_vel_z):
    """
    Calculate BrIC (Brain Injury Criterion)
    Critical angular velocities (rad/s) from literature:
    ωxc = 66.25 rad/s, ωyc = 56.45 rad/s, ωzc = 42.87 rad/s
    """
    omega_xc = 66.25  # Critical angular velocity for x-axis
    omega_yc = 56.45  # Critical angular velocity for y-axis
    omega_zc = 42.87  # Critical angular velocity for z-axis
    
    # Get maximum absolute angular velocities
    max_omega_x = np.max(np.abs(rot_vel_x))
    max_omega_y = np.max(np.abs(rot_vel_y))
    max_omega_z = np.max(np.abs(rot_vel_z))
    
    # Calculate BrIC components
    bric = np.sqrt((max_omega_x/omega_xc)**2 + 
                   (max_omega_y/omega_yc)**2 + 
                   (max_omega_z/omega_zc)**2)
    
    return bric

def calculate_vt_star_risk(peak_linear_acc, peak_rotational_acc):
    """
    Calculate Virginia Tech STAR methodology risk of concussion
    peak_linear_acc: Peak Linear Acceleration in g's
    peak_rotational_acc: Peak Rotational Acceleration in rad/s²
    Returns: Probability of concussion (0-1)
    """
    # VT STAR risk function coefficients
    β0 = -10.2
    β1 = 0.0433
    β2 = 0.000873
    β3 = -0.00000092
    
    # Calculate risk using VT STAR equation
    exponent = -(β0 + β1*peak_linear_acc + β2*peak_rotational_acc + 
                β3*peak_linear_acc*peak_rotational_acc)
    risk = 1 / (1 + np.exp(exponent))
    
    return risk

def process_file(args):
    """Process a single Excel file with motion and rotation component filtering"""
    file_path, config = args
    try:
        # Read the Excel file
        df = pd.read_excel(file_path)
        
        # Apply zeroing based on configuration
        if config['zero_x']:
            df['LinAccX'] = 0
            df['LinVelX'] = 0
        if config['zero_y']:
            df['LinAccY'] = 0
            df['LinVelY'] = 0
        if config['zero_z']:
            df['LinAccZ'] = 0
            df['LinVelZ'] = 0
        if config['zero_roll']:
            df['RotVelX'] = 0
            df['RotAccX'] = 0
        if config['zero_pitch']:
            df['RotVelY'] = 0
            df['RotAccY'] = 0
        if config['zero_yaw']:
            df['RotVelZ'] = 0
            df['RotAccZ'] = 0
            
        # Recalculate resultants after zeroing
        if any([config['zero_x'], config['zero_y'], config['zero_z']]):
            df['LinAccRes'] = np.sqrt(df['LinAccX']**2 + df['LinAccY']**2 + df['LinAccZ']**2)
        if any([config['zero_roll'], config['zero_pitch'], config['zero_yaw']]):
            df['RotAccRes'] = np.sqrt(df['RotAccX']**2 + df['RotAccY']**2 + df['RotAccZ']**2)
        
        # Calculate HIC15 and HIC36
        hic15, hic15_start, hic15_end = calculate_hic(df['LinAccRes'].values, df['t(ms)'].values, 15)
        hic36, hic36_start, hic36_end = calculate_hic(df['LinAccRes'].values, df['t(ms)'].values, 36)
        
        # Calculate BrIC
        bric = calculate_bric(df['RotVelX'].values, df['RotVelY'].values, df['RotVelZ'].values)
        
        # Calculate maximum angles
        max_angle_x = calculate_angle(df['RotVelX'].values, df['t(ms)'].values)
        max_angle_y = calculate_angle(df['RotVelY'].values, df['t(ms)'].values)
        max_angle_z = calculate_angle(df['RotVelZ'].values, df['t(ms)'].values)
        
        # Calculate maximum displacements
        max_disp_x = calculate_displacement(df['LinAccX'].values, df['t(ms)'].values)
        max_disp_y = calculate_displacement(df['LinAccY'].values, df['t(ms)'].values)
        max_disp_z = calculate_displacement(df['LinAccZ'].values, df['t(ms)'].values)
        
        # Get peak velocities (already in data)
        peak_lin_vel_x = get_peak_with_sign(df['LinVelX']) if 'LinVelX' in df.columns else None
        peak_lin_vel_y = get_peak_with_sign(df['LinVelY']) if 'LinVelY' in df.columns else None
        peak_lin_vel_z = get_peak_with_sign(df['LinVelZ']) if 'LinVelZ' in df.columns else None
        
        # Get peak angular velocities
        peak_ang_vel_x = get_peak_with_sign(df['RotVelX'])
        peak_ang_vel_y = get_peak_with_sign(df['RotVelY'])
        peak_ang_vel_z = get_peak_with_sign(df['RotVelZ'])
        
        # Get peak values for VT STAR calculation
        peak_linear_acc = df['LinAccRes'].max()
        peak_rotational_acc = df['RotAccRes'].max()
        
        # Calculate VT STAR risk
        concussion_risk = calculate_vt_star_risk(peak_linear_acc, peak_rotational_acc)
        
        # Get filename without path
        filename = os.path.basename(file_path)
        
        # Calculate peak values
        peak_values = {
            'Filename': filename,
            'Peak Linear Acceleration Resultant (g)': peak_linear_acc,
            'Peak Linear Acceleration X (g)': get_peak_with_sign(df['LinAccX']),
            'Peak Linear Acceleration Y (g)': get_peak_with_sign(df['LinAccY']),
            'Peak Linear Acceleration Z (g)': get_peak_with_sign(df['LinAccZ']),
            'Peak Rotational Acceleration Resultant (rad/s²)': peak_rotational_acc,
            'Peak Rotational Acceleration X (rad/s²)': get_peak_with_sign(df['RotAccX']),
            'Peak Rotational Acceleration Y (rad/s²)': get_peak_with_sign(df['RotAccY']),
            'Peak Rotational Acceleration Z (rad/s²)': get_peak_with_sign(df['RotAccZ']),
            'Maximum Angle X (degrees)': max_angle_x,
            'Maximum Angle Y (degrees)': max_angle_y,
            'Maximum Angle Z (degrees)': max_angle_z,
            'Maximum Displacement X (m)': max_disp_x,
            'Maximum Displacement Y (m)': max_disp_y,
            'Maximum Displacement Z (m)': max_disp_z,
            'Peak Linear Velocity X (m/s)': peak_lin_vel_x,
            'Peak Linear Velocity Y (m/s)': peak_lin_vel_y,
            'Peak Linear Velocity Z (m/s)': peak_lin_vel_z,
            'Peak Angular Velocity X (rad/s)': peak_ang_vel_x,
            'Peak Angular Velocity Y (rad/s)': peak_ang_vel_y,
            'Peak Angular Velocity Z (rad/s)': peak_ang_vel_z,
            'HIC15': hic15,
            'HIC15 Time Window (ms)': f"{hic15_start:.1f} - {hic15_end:.1f}",
            'HIC36': hic36,
            'HIC36 Time Window (ms)': f"{hic36_start:.1f} - {hic36_end:.1f}",
            'BrIC': bric,
            'Concussion Risk Probability': concussion_risk  # Store as number for percentile calculation
        }
        
        # Create a new dataframe with peak values
        peak_df = pd.DataFrame([peak_values])
        
        # Read the existing Excel file with openpyxl engine
        with pd.ExcelWriter(file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
            # Format concussion risk as percentage for individual file
            peak_df['Concussion Risk Probability'] = peak_df['Concussion Risk Probability'].map('{:.3%}'.format)
            peak_df.to_excel(writer, sheet_name='Peak Values', index=False)
            
        return peak_values
    except Exception as e:
        print(f"Error processing {file_path}: {str(e)}")
        return None

def create_summary_file(all_results, output_path):
    """
    Create a summary Excel file with all results and percentile rankings
    """
    # Convert results to DataFrame
    summary_df = pd.DataFrame(all_results)
    
    # List of numerical columns for percentile calculation
    numerical_columns = [
        'Peak Linear Acceleration Resultant (g)',
        'Peak Linear Acceleration X (g)',
        'Peak Linear Acceleration Y (g)',
        'Peak Linear Acceleration Z (g)',
        'Peak Rotational Acceleration Resultant (rad/s²)',
        'Peak Rotational Acceleration X (rad/s²)',
        'Peak Rotational Acceleration Y (rad/s²)',
        'Peak Rotational Acceleration Z (rad/s²)',
        'Maximum Angle X (degrees)',
        'Maximum Angle Y (degrees)',
        'Maximum Angle Z (degrees)',
        'Maximum Displacement X (m)',
        'Maximum Displacement Y (m)',
        'Maximum Displacement Z (m)',
        'Peak Linear Velocity X (m/s)',
        'Peak Linear Velocity Y (m/s)',
        'Peak Linear Velocity Z (m/s)',
        'Peak Angular Velocity X (rad/s)',
        'Peak Angular Velocity Y (rad/s)',
        'Peak Angular Velocity Z (rad/s)',
        'HIC15',
        'HIC36',
        'BrIC',
        'Concussion Risk Probability'
    ]
    
    # Calculate percentiles for each numerical column
    for col in numerical_columns:
        if col in summary_df.columns:
            percentile_col = f'{col} Percentile'
            summary_df[percentile_col] = summary_df[col].rank(pct=True) * 100
            
    # Format the concussion risk probability as percentage
    summary_df['Concussion Risk Probability'] = summary_df['Concussion Risk Probability'].map('{:.3%}'.format)
    
    # Sort by concussion risk probability (descending)
    summary_df = summary_df.sort_values('Concussion Risk Probability', ascending=False)
    
    # Create Excel writer
    with pd.ExcelWriter(output_path, engine='openpyxl') as writer:
        # Write main data
        summary_df.to_excel(writer, sheet_name='All Results', index=False)
        
        # Create sheets for different percentile ranges
        percentile_ranges = [(0, 25), (25, 50), (50, 75), (75, 100)]
        for start, end in percentile_ranges:
            sheet_name = f'{start}-{end} Percentile'
            mask = (summary_df['Concussion Risk Probability Percentile'] >= start) & \
                  (summary_df['Concussion Risk Probability Percentile'] < end)
            percentile_df = summary_df[mask]
            if not percentile_df.empty:
                percentile_df.to_excel(writer, sheet_name=sheet_name, index=False)

def main():
    # Get the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Get all Excel files in the directory
    excel_files = [f for f in os.listdir(current_dir) if f.endswith('.xlsx') and f != 'summary_results.xlsx']
    file_paths = [os.path.join(current_dir, f) for f in excel_files]
    
    # Get user configuration for motion and rotation components
    print("\nSelect which motion components to zero out (y/n for each):")
    config = {
        'zero_x': input("Zero out X translation? (y/n): ").lower() == 'y',
        'zero_y': input("Zero out Y translation? (y/n): ").lower() == 'y',
        'zero_z': input("Zero out Z translation? (y/n): ").lower() == 'y',
        'zero_roll': input("Zero out Roll rotation? (y/n): ").lower() == 'y',
        'zero_pitch': input("Zero out Pitch rotation? (y/n): ").lower() == 'y',
        'zero_yaw': input("Zero out Yaw rotation? (y/n): ").lower() == 'y'
    }
    
    # Create process arguments with configuration
    process_args = [(f, config) for f in file_paths]
    
    # Create a pool of workers
    num_processes = min(cpu_count(), 8)  # Use up to 8 cores
    
    # Create output filename with configuration
    zeroed_components = []
    if config['zero_x']: zeroed_components.append('X')
    if config['zero_y']: zeroed_components.append('Y')
    if config['zero_z']: zeroed_components.append('Z')
    if config['zero_roll']: zeroed_components.append('Roll')
    if config['zero_pitch']: zeroed_components.append('Pitch')
    if config['zero_yaw']: zeroed_components.append('Yaw')
    
    config_suffix = f"_no{'_'.join(zeroed_components)}" if zeroed_components else ""
    summary_filename = f'summary_results{config_suffix}.xlsx'
    
    print(f"\nProcessing {len(file_paths)} files using {num_processes} processes...")
    print("Components zeroed:", ', '.join(zeroed_components) if zeroed_components else "None")
    
    # Process files in parallel with progress bar
    with Pool(num_processes) as pool:
        results = list(tqdm(pool.imap(lambda x: process_file(x), process_args), total=len(file_paths)))
    
    # Filter out None results (failed processing)
    results = [r for r in results if r is not None]
    
    # Create summary file
    summary_path = os.path.join(current_dir, summary_filename)
    create_summary_file(results, summary_path)
    
    # Print summary
    successful = len(results)
    print(f"\nProcessing complete!")
    print(f"Successfully processed: {successful} files")
    print(f"Failed: {len(file_paths) - successful} files")
    print(f"Summary file created: {summary_path}")

if __name__ == '__main__':
    main()