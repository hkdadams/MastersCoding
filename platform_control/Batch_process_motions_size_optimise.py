##### WARNING: THIS SCRIPT IS DESIGNED TO RUN IN A MULTIPROCESSING ENVIRONMENT #####
##### IT CONTAINS AN EMBEDDED OPTIMIZATION ALGORITHM FOR RAIL AND LEG LENGTHS, DESIGNED TO BE RELIABLE BUT NOT FAST #####
##### THIS TAKES A SIGNIFICANT AMOUNT OF TIME TO RUN UNLESS YOUR COMPUTER HAS A LARGE NUMBER OF CORES DO NOT RUN#####

import sys
import os
import pandas as pd
import numpy as np
from tqdm import tqdm
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed

# Add the parent directory (MASTERS) to the Python path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from Rugby_head_motions.process_kinematics import calculate_position_orientation
from platform_controllerMP import PlatformController  # Using the MP version

def process_file(args):
    """Process a single Excel file with combined rail and leg length optimization"""
    file_path, zero_config, slider_min_travel_offset = args
    try:
        # Read the Excel file
        df = pd.read_excel(file_path)
        
        # Calculate position and orientation with zeroing configuration
        results_df = calculate_position_orientation(df, zero_config)
        
        # Write results to Position_Orientation sheet
        with pd.ExcelWriter(file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
            results_df.to_excel(writer, sheet_name='Position_Orientation', index=False)
        
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
        
        print(f"\nProcessing file: {os.path.basename(file_path)}")
        print("Initial platform configuration:")
        print(f"Position (x,y,z): ({trajectory_df['x'].iloc[0]:.3f}, {trajectory_df['y'].iloc[0]:.3f}, {trajectory_df['z'].iloc[0]:.3f})m")
        print(f"Rotation (r,p,y): ({trajectory_df['roll'].iloc[0]:.1f}, {trajectory_df['pitch'].iloc[0]:.1f}, {trajectory_df['yaw'].iloc[0]:.1f})°")
        
        def test_configuration(leg_length, rail_length, current_slider_min_travel_offset):
            """Helper function to test a specific configuration"""
            try:
                # Create controller with current configuration
                # Ensure log_file_path is defined or passed appropriately if needed by PlatformController
                # For this example, assuming a generic or temp log path for optimization runs,
                # or that PlatformController handles a None path.
                log_file_path_for_test = os.path.join(os.path.dirname(file_path), "platform_outputs", f"debug_log_opt_{os.path.basename(file_path).replace('.xlsx','')}.txt")
                os.makedirs(os.path.dirname(log_file_path_for_test), exist_ok=True)

                controller = PlatformController(
                    leg_length=leg_length, 
                    rail_max_travel=rail_length,
                    slider_min_travel_offset=current_slider_min_travel_offset,
                    log_file_path=log_file_path_for_test, 
                    log_attempts=False
                )
                
                # Process without optimization to get baseline
                results_df_no_opt = controller.process_trajectory(
                    trajectory_df.copy(), # Use a copy to avoid modification issues
                    file_path=file_path, # Pass file_path for potential internal use by process_trajectory
                    apply_optimization=False,
                    auto_overwrite=True # Assuming auto-overwrite for these internal tests
                )
                if results_df_no_opt is None: return None
                peak_velocities_no_opt = results_df_no_opt['velocities'].apply(lambda x: np.max(np.abs(x))).max()
                peak_accelerations_no_opt = results_df_no_opt['accelerations'].apply(lambda x: np.max(np.abs(x))).max()

                # Optimize offsets
                pos_offset, rot_offset = controller.optimize_offsets(
                    trajectory_df.copy(), 
                    log_file_path=log_file_path_for_test # Use the specific log for this test
                )
                if pos_offset is None or rot_offset is None: return None
                
                # Process with optimization
                results_df_opt = controller.process_trajectory(
                    trajectory_df.copy(), 
                    file_path=file_path,
                    apply_optimization=True, # Apply the just-found offsets
                    auto_overwrite=True
                )
                if results_df_opt is None: return None
                
                peak_velocities_opt = results_df_opt['velocities'].apply(lambda x: np.max(np.abs(x))).max()
                peak_accelerations_opt = results_df_opt['accelerations'].apply(lambda x: np.max(np.abs(x))).max()
                peak_torque_opt = results_df_opt['motor_torque_slider1'].abs().max() # Get absolute peak torque

                score = controller.best_feasible_solution['score'] if controller.best_feasible_solution else float('inf')

                return {
                    'leg_length': leg_length,
                    'rail_length': rail_length,
                    'slider_min_travel_offset': current_slider_min_travel_offset,
                    'score': score,
                    'peak_velocities_no_opt': peak_velocities_no_opt,
                    'peak_accelerations_no_opt': peak_accelerations_no_opt,
                    'peak_velocities': peak_velocities_opt,
                    'peak_accelerations': peak_accelerations_opt,
                    'peak_torque_slider1': peak_torque_opt, # Store peak torque
                    'pos_offset': pos_offset,
                    'rot_offset': rot_offset
                }
            except Exception as e:
                print(f"Error in test_configuration (L:{leg_length:.2f} R:{rail_length:.2f} O:{current_slider_min_travel_offset:.2f}): {e}")
                return None
        
        # Test each rail length with two-phase leg length optimization
        rail_lengths = [0.5, 0.6, 0.75]  # Specific rail lengths to test
        all_configs = []
        
        for rail_length in rail_lengths:
            print(f"\nTesting rail length: {rail_length}m")
            
            # Phase 1: Test coarse leg length increments
            print("Phase 1: Testing coarse leg length increments...")
            coarse_lengths = np.arange(0.2, 0.76, 0.1)  # 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
            phase1_results = []
            
            for leg_length in coarse_lengths:
                print(f"\nTrying leg length: {leg_length:.2f}m")
                result = test_configuration(leg_length, rail_length, slider_min_travel_offset)
                if result and result['feasible']:
                    phase1_results.append(result)
                    print(f"Score: {result['score']:.3f}")
            
            if not phase1_results:
                print(f"No feasible configurations found for rail length {rail_length}m")
                continue
            
            # Find best coarse result for this rail length
            best_coarse = min(phase1_results, key=lambda x: x['score'])
            best_coarse_length = best_coarse['leg_length']
            
            # Phase 2: Fine-tune around best coarse result
            print(f"\nPhase 2: Fine-tuning around leg length {best_coarse_length:.2f}m...")
            fine_range = 0.1  # Test ±0.1m around best coarse result
            fine_lengths = np.arange(
                max(0.2, best_coarse_length - fine_range),
                min(0.75, best_coarse_length + fine_range + 0.05),
                0.05
            )
            
            phase2_results = []
            for leg_length in fine_lengths:
                if leg_length in coarse_lengths:  # Skip if already tested
                    continue
                print(f"\nTrying leg length: {leg_length:.2f}m")
                result = test_configuration(leg_length, rail_length, slider_min_travel_offset)
                if result and result['feasible']:
                    phase2_results.append(result)
                    print(f"Score: {result['score']:.3f}")
            
            # Combine results for this rail length
            rail_results = phase1_results + phase2_results
            if rail_results:
                all_configs.extend(rail_results)
        
        if not all_configs:
            print(f"No feasible configurations found for {os.path.basename(file_path)}") # Added print
            return False, (file_path, "No feasible configurations found") # Modified return
        
        # Find best overall configuration
        best_config = min(all_configs, key=lambda x: x['score'])
        
        print("\\nBest configuration found:")
        print(f"  File: {os.path.basename(file_path)}") # Added file
        print(f"  Rail length: {best_config['rail_length']:.3f}m")
        print(f"  Leg length: {best_config['leg_length']:.3f}m")
        print(f"  Slider offset: {best_config['slider_min_travel_offset']:.3f}m") # Added offset
        print(f"  Score: {best_config['score']:.3f}") # Added score
        print(f"  Peak velocity (opt): {best_config['peak_velocities']:.3f} m/s (No-opt: {best_config['peak_velocities_no_opt']:.3f} m/s)")
        print(f"  Peak acceleration (opt): {best_config['peak_accelerations']:.3f} m/s² (No-opt: {best_config['peak_accelerations_no_opt']:.3f} m/s²)")
        print(f"  Peak Torque Slider 1 (opt): {best_config['peak_torque_slider1']:.3f} Nm") # Added torque
        print(f"  Position offset: ({best_config['pos_offset'][0]:.3f}, {best_config['pos_offset'][1]:.3f}, {best_config['pos_offset'][2]:.3f})m")
        print(f"  Rotation offset: ({best_config['rot_offset'][0]:.1f}, {best_config['rot_offset'][1]:.1f}, {best_config['rot_offset'][2]:.1f})°")
        
        # Calculate improvements
        # Handle potential division by zero if no_opt values are zero
        vel_improvement = 0
        if best_config['peak_velocities_no_opt'] != 0:
             vel_improvement = (best_config['peak_velocities_no_opt'] - best_config['peak_velocities']) / best_config['peak_velocities_no_opt'] * 100
        
        acc_improvement = 0
        if best_config['peak_accelerations_no_opt'] != 0:
            acc_improvement = (best_config['peak_accelerations_no_opt'] - best_config['peak_accelerations']) / best_config['peak_accelerations_no_opt'] * 100
        
        print("\\nImprovements with optimal configuration:")
        print(f"  Velocity reduction: {vel_improvement:.1f}%")
        print(f"  Acceleration reduction: {acc_improvement:.1f}%")
        
        # Prepare data for summary DataFrame
        summary_data = best_config.copy() # Start with all data from best_config
        summary_data['file_name'] = os.path.basename(file_path)
        summary_data['vel_improvement'] = vel_improvement
        summary_data['acc_improvement'] = acc_improvement
        # Convert numpy arrays in pos_offset and rot_offset to tuples for easier DataFrame handling
        summary_data['pos_offset'] = tuple(summary_data['pos_offset'])
        summary_data['rot_offset'] = tuple(summary_data['rot_offset'])


        return True, summary_data # Return the prepared dictionary
        
    except Exception as e:
        # Ensure a consistent return type for errors
        return False, (file_path, f"Error processing {os.path.basename(file_path)}: {str(e)}")

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
    
    # Get slider_min_travel_offset
    slider_min_travel_offset = float(input("\nEnter slider minimum travel offset in meters (default 0.0): ") or "0.0")
    print(f"Using slider minimum travel offset: {slider_min_travel_offset:.3f}m")

    # Create arguments for each file (note: rail_max_travel is handled inside process_file for optimization)
    process_args = [(f, zero_config, slider_min_travel_offset) for f in file_paths]
    
    # Use half the available cores to avoid system overload
    num_processes = max(1, mp.cpu_count() // 2)
    
    print(f"\nProcessing {len(file_paths)} files using {num_processes} processes...")
    
    # Process files in parallel with progress bar
    successful_count = 0 # Renamed from successful to avoid conflict
    failed_count = 0     # Renamed from failed
    optimal_configs_list = [] # Renamed from optimal_configs to avoid conflict with module
    failed_files_info = [] # To store info about failed files

    with ProcessPoolExecutor(max_workers=num_processes) as executor:
        futures = [executor.submit(process_file, args) for args in process_args]
        
        for future in tqdm(as_completed(futures), total=len(futures), 
                         desc="Processing files", unit="file"):
            success, result = future.result()
            if success:
                optimal_configs_list.append(result) # result is now the summary_data dict
                successful_count += 1
            else:
                failed_count += 1
                # result is (file_path, error_message)
                failed_files_info.append(result) 
                print(f"Failed: {result[0]} - {result[1]}")


    # Create summary DataFrame
    if optimal_configs_list:
        summary_df = pd.DataFrame(optimal_configs_list) # optimal_configs_list contains dicts
        
        # Define columns for the summary Excel, including the new torque column
        summary_columns = [
            'file_name', 'leg_length', 'rail_length', 'slider_min_travel_offset', 'score',
            'peak_velocities_no_opt', 'peak_accelerations_no_opt',
            'peak_velocities', 'peak_accelerations', 'peak_torque_slider1',
            'vel_improvement', 'acc_improvement',
            'pos_offset', 'rot_offset'
        ]
        summary_df = summary_df[summary_columns] # Ensure correct column order and selection

        # Calculate overall statistics
        avg_leg_length = summary_df['leg_length'].mean() # Renamed for clarity
        std_leg_length = summary_df['leg_length'].std()  # Renamed for clarity
        mode_leg_length = summary_df['leg_length'].mode()
        mode_leg_length_val = mode_leg_length.iloc[0] if not mode_leg_length.empty else np.nan # Handle empty mode

        avg_rail_length = summary_df['rail_length'].mean()
        std_rail_length = summary_df['rail_length'].std()
        mode_rail_length = summary_df['rail_length'].mode()
        mode_rail_length_val = mode_rail_length.iloc[0] if not mode_rail_length.empty else np.nan

        avg_slider_offset = summary_df['slider_min_travel_offset'].mean()
        std_slider_offset = summary_df['slider_min_travel_offset'].std()
        mode_slider_offset = summary_df['slider_min_travel_offset'].mode()
        mode_slider_offset_val = mode_slider_offset.iloc[0] if not mode_slider_offset.empty else np.nan

        avg_vel_improvement = summary_df['vel_improvement'].mean() # Renamed for clarity
        avg_acc_improvement = summary_df['acc_improvement'].mean()
        avg_peak_torque = summary_df['peak_torque_slider1'].mean()


        print("\\nOptimization Summary Statistics:")
        print(f"  Average optimal leg length: {avg_leg_length:.3f}m (±{std_leg_length:.3f}m), Mode: {mode_leg_length_val:.3f}m")
        print(f"  Average optimal rail length: {avg_rail_length:.3f}m (±{std_rail_length:.3f}m), Mode: {mode_rail_length_val:.3f}m")
        print(f"  Average optimal slider offset: {avg_slider_offset:.3f}m (±{std_slider_offset:.3f}m), Mode: {mode_slider_offset_val:.3f}m")
        print(f"  Average velocity improvement: {avg_vel_improvement:.1f}%")
        print(f"  Average acceleration improvement: {avg_acc_improvement:.1f}%")
        print(f"  Average peak torque (Slider 1): {avg_peak_torque:.3f} Nm")
        
        # Save detailed results
        summary_file = os.path.join(dir_path, "size_optimization_summary_results.xlsx") # Changed filename
        with pd.ExcelWriter(summary_file) as writer:
            summary_df.to_excel(writer, sheet_name='Optimization_Summary', index=False)
        
        print(f"\\nDetailed summary results saved to: {summary_file}")
    
    # Print summary
    print(f"\\nProcessing complete!")
    print(f"Successfully processed: {successful_count} files") # Use renamed counter
    print(f"Failed: {failed_count} files")                   # Use renamed counter
    if failed_files_info:
        print("\\nDetails of failed files:")
        for f_path, err_msg in failed_files_info:
            print(f"  - {os.path.basename(f_path)}: {err_msg}")

if __name__ == '__main__':
    main()
