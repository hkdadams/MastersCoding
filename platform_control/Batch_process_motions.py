import sys
import os
import pandas as pd
from tqdm import tqdm
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed

# Add the parent directory (MASTERS) to the Python path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from Rugby_head_motions.process_kinematics import calculate_position_orientation
from platform_controllerMP import PlatformController  # Using the MP version

def process_file(args):
    """Process a single Excel file to calculate position and orientation"""
    file_path, leg_length, rail_max_travel, zero_config = args
    try:
        # Read the Excel file
        df = pd.read_excel(file_path)
        
        # Calculate position and orientation with zeroing configuration
        results_df = calculate_position_orientation(df, zero_config)
        
        # Write results to Position_Orientation sheet
        with pd.ExcelWriter(file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
            results_df.to_excel(writer, sheet_name='Position_Orientation', index=False)
        
        # Create platform controller
        controller = PlatformController(leg_length, rail_max_travel)
        
        # Process trajectory with the controller
        trajectory_df = pd.read_excel(file_path, sheet_name='Position_Orientation')
        
        # Rename columns to match controller expectations
        column_map = {
            't(ms)': 'time',
            'X(m)': 'x',
            'Y(m)': 'y',
            'Z(m)': 'z',
            'Roll(deg)': 'roll',
            'Pitch(deg)': 'pitch',
            'Yaw(deg)': 'yaw'
        }
        trajectory_df = trajectory_df.rename(columns=column_map)
        trajectory_df['time'] = trajectory_df['time'] / 1000.0  # Convert ms to seconds
        
        # Process trajectory without optimization first
        print(f"\nProcessing file: {os.path.basename(file_path)}")
        print("Initial platform configuration:")
        print(f"Position (x,y,z): ({trajectory_df['x'].iloc[0]:.3f}, {trajectory_df['y'].iloc[0]:.3f}, {trajectory_df['z'].iloc[0]:.3f})m")
        print(f"Rotation (r,p,y): ({trajectory_df['roll'].iloc[0]:.1f}, {trajectory_df['pitch'].iloc[0]:.1f}, {trajectory_df['yaw'].iloc[0]:.1f})°")
        
        # Process without optimization first
        results_df_no_opt = controller.process_trajectory(trajectory_df, file_path=file_path, apply_optimization=False)
        
        # Calculate initial statistics
        peak_velocities_no_opt = max(max(abs(v)) for v in results_df_no_opt['velocities'])
        peak_accelerations_no_opt = max(max(abs(a)) for a in results_df_no_opt['accelerations'])
        
        print("\nNon-optimized results:")
        print(f"Peak velocity: {peak_velocities_no_opt:.3f} m/s")
        print(f"Peak acceleration: {peak_accelerations_no_opt:.3f} m/s²")
        
        # Optimize offsets
        print("\nOptimizing trajectory offsets...")
        pos_offset, rot_offset = controller.optimize_offsets(trajectory_df)
        
        # Set optimized offsets
        controller.position_offset = pos_offset
        controller.rotation_offset = rot_offset
        
        # Process with optimization
        print("\nProcessing trajectory with optimized offsets...")
        results_df = controller.process_trajectory(trajectory_df, file_path=file_path)
        
        # Calculate improvement
        peak_velocities = max(max(abs(v)) for v in results_df['velocities'])
        peak_accelerations = max(max(abs(a)) for a in results_df['accelerations'])
        
        vel_improvement = (peak_velocities_no_opt - peak_velocities) / peak_velocities_no_opt * 100
        acc_improvement = (peak_accelerations_no_opt - peak_accelerations) / peak_accelerations_no_opt * 100
        
        print("\nImprovement:")
        print(f"Velocity reduction: {vel_improvement:.1f}%")
        print(f"Acceleration reduction: {acc_improvement:.1f}%")
        
        return True, file_path
        
    except Exception as e:
        return False, f"Error processing {file_path}: {str(e)}"

def main():
    # Ask for the directory path
    while True:
        dir_path = input("Enter the path to the folder containing Excel files (press Enter for current directory): ").strip()
        if not dir_path:  # If empty, use current directory
            dir_path = os.path.dirname(os.path.abspath(__file__))
            break
        if os.path.isdir(dir_path):
            break
        print(f"Error: '{dir_path}' is not a valid directory. Please try again.")
    
    # Get all Excel files in the directory
    excel_files = [f for f in os.listdir(dir_path) if f.endswith('.xlsx')]
    if not excel_files:
        print(f"No Excel files found in {dir_path}")
        return
        
    print(f"\nFound {len(excel_files)} Excel files in {dir_path}")
    file_paths = [os.path.join(dir_path, f) for f in excel_files]
    
    # Get zeroing configuration
    print("\nSelect which motion components to zero out:")
    zero_config = {
        'zero_x': input("Zero out X translation? (y/n, default=n): ").lower().startswith('y'),
        'zero_y': input("Zero out Y translation? (y/n, default=n): ").lower().startswith('y'),
        'zero_z': input("Zero out Z translation? (y/n, default=n): ").lower().startswith('y'),
        'zero_roll': input("Zero out Roll rotation? (y/n, default=n): ").lower().startswith('y'),
        'zero_pitch': input("Zero out Pitch rotation? (y/n, default=n): ").lower().startswith('y'),
        'zero_yaw': input("Zero out Yaw rotation? (y/n, default=n): ").lower().startswith('y')
    }
    
    # Print selected zeroing configuration
    print("\nSelected zeroing configuration:")
    for component, is_zeroed in zero_config.items():
        print(f"{component}: {'Yes' if is_zeroed else 'No'}")
    
    # Get geometric parameters
    leg_length = float(input("\nEnter leg length in meters (default 0.3): ") or "0.3")
    rail_max_travel = float(input("Enter maximum rail travel in meters (default 0.5): ") or "0.5")
    
    # Create arguments for each file
    process_args = [(f, leg_length, rail_max_travel, zero_config) for f in file_paths]
    
    # Use half the available cores to avoid system overload
    num_processes = max(1, mp.cpu_count() // 2)
    
    print(f"\nProcessing {len(file_paths)} files using {num_processes} processes...")
    
    # Process files in parallel with progress bar
    successful = 0
    failed = 0
    with ProcessPoolExecutor(max_workers=num_processes) as executor:
        futures = [executor.submit(process_file, args) for args in process_args]
        
        for future in tqdm(as_completed(futures), total=len(futures), 
                         desc="Processing files", unit="file"):
            success, result = future.result()
            if success:
                successful += 1
            else:
                failed += 1
                print(f"\n{result}")
    
    # Print summary
    print(f"\nProcessing complete!")
    print(f"Successfully processed: {successful} files")
    print(f"Failed: {failed} files")

if __name__ == '__main__':
    main()
