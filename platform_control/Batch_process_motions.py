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
    file_path, leg_length, rail_max_travel, zero_config, enable_logging = args
    error_details = {}
    
    # Create output directory structure
    output_dir = os.path.join(os.path.dirname(file_path), "platform_outputs")
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Create debug log file path for this file
        input_file_name = os.path.basename(file_path).replace('.xlsx', '')
        log_file_path = os.path.join(output_dir, f"debug_log_{input_file_name}.txt")

        # Read the Excel file
        df = pd.read_excel(file_path)
        
        # Calculate position and orientation with zeroing configuration
        try:
            results_df = calculate_position_orientation(df, zero_config)
        except Exception as e:
            error_details['position_orientation_error'] = str(e)
            raise ValueError(f"Failed to calculate position and orientation: {str(e)}")
        
        # Write results to Position_Orientation sheet
        try:
            with pd.ExcelWriter(file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
                results_df.to_excel(writer, sheet_name='Position_Orientation', index=False)
        except Exception as e:
            error_details['excel_write_error'] = str(e)
            raise ValueError(f"Failed to write results to Excel: {str(e)}")
        
        # Create platform controller with logging options
        controller = PlatformController(leg_length, rail_max_travel, log_file_path, log_attempts=enable_logging)
        
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
        
        # Optimize offsets with proper parameters
        print("\nOptimizing trajectory offsets...")
        pos_offset, rot_offset = controller.optimize_offsets(
            trajectory_df=trajectory_df,
            log_file_path=log_file_path,
            num_cores=None  # Will use default value
        )
        
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
        
        return True, file_path, None
        
    except Exception as e:
        error_details['main_error'] = str(e)
        error_msg = f"Error processing {os.path.basename(file_path)}:\n"
        error_msg += "\n".join([f"{k}: {v}" for k, v in error_details.items()])
        
        # Log error to a failure log file in the output directory
        error_log_path = os.path.join(output_dir, "processing_errors.log")
        try:
            with open(error_log_path, 'a') as error_log:
                error_log.write(f"\n{'='*50}\n")
                error_log.write(f"File: {file_path}\n")
                error_log.write(f"Time: {pd.Timestamp.now()}\n")
                error_log.write(f"Error Details:\n")
                for k, v in error_details.items():
                    error_log.write(f"{k}:\n{v}\n")
                error_log.write(f"{'='*50}\n")
        except Exception as log_error:
            print(f"Failed to write to error log: {str(log_error)}")
            
        return False, file_path, error_details

def check_existing_outputs(file_paths):
    """Check for existing output files and get user permission to overwrite"""
    existing_files = []
    
    for file_path in file_paths:
        output_dir = os.path.join(os.path.dirname(file_path), "platform_outputs")
        input_file_name = os.path.basename(file_path).replace('.xlsx', '')
        expected_output = os.path.join(output_dir, f"{input_file_name}_platform_motion.xlsx")
        
        if os.path.exists(expected_output):
            existing_files.append(expected_output)
    
    if existing_files:
        print("\nThe following output files already exist:")
        for file in existing_files:
            print(f"  - {os.path.basename(file)}")
        
        user_input = input("\nDo you want to overwrite these files? (y/N): ")
        if not user_input.lower().startswith('y'):
            print("Operation cancelled. Please remove existing files or choose different input files.")
            return False
        print("\nProceeding with file processing - existing files will be overwritten.")
        return True
    
    return True  # No existing files found, proceed normally

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
    
    # Add logging option
    enable_logging = input("\nEnable optimization logging? (y/N): ").lower().startswith('y')
    
    # Check for existing output files and get permission to overwrite
    if not check_existing_outputs(file_paths):
        return
        
    # Create arguments for each file
    process_args = [(f, leg_length, rail_max_travel, zero_config, enable_logging) for f in file_paths]
    
    print(f"\nProcessing {len(file_paths)} files sequentially...")
    
    # Process files one at a time with progress bar
    successful = 0
    failed = 0
    failed_files = []
    
    # Create a progress bar for sequential processing
    for args in tqdm(process_args, desc="Processing files", unit="file"):
        success, file_path, error_details = process_file(args)
        if success:
            successful += 1
        else:
            failed += 1
            failed_files.append((file_path, error_details))
            print(f"\nError processing {os.path.basename(file_path)}:")
            for error_type, error_msg in error_details.items():
                print(f"  {error_type}: {error_msg}")
    
    # Print summary
    print(f"\nProcessing complete!")
    print(f"Successfully processed: {successful} files")
    print(f"Failed: {failed} files")
    
    if failed > 0:
        print("\nFailed files summary:")
        for file_path, error_details in failed_files:
            print(f"\n{os.path.basename(file_path)}:")
            for error_type, error_msg in error_details.items():
                print(f"  {error_type}: {error_msg}")
        
        # Point to the error log in the output directory
        output_dir = os.path.join(os.path.dirname(file_paths[0]), "platform_outputs")
        print(f"\nDetailed error logs have been written to: {os.path.join(output_dir, 'processing_errors.log')}")

if __name__ == '__main__':
    main()
