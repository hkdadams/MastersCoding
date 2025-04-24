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
    file_path, zero_config = args
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
        
        def test_configuration(leg_length, rail_length):
            """Helper function to test a specific configuration"""
            try:
                # Create controller with current parameters
                controller = PlatformController(leg_length, rail_length)
                
                # Process without optimization first to get baseline
                results_df_no_opt = controller.process_trajectory(trajectory_df, file_path=file_path, apply_optimization=False)
                peak_velocities_no_opt = max(max(abs(v)) for v in results_df_no_opt['velocities'])
                peak_accelerations_no_opt = max(max(abs(a)) for a in results_df_no_opt['accelerations'])
                
                # Optimize offsets for this configuration
                pos_offset, rot_offset = controller.optimize_offsets(trajectory_df)
                
                # Set optimized offsets
                controller.position_offset = pos_offset
                controller.rotation_offset = rot_offset
                
                # Process with optimization
                results_df = controller.process_trajectory(trajectory_df, file_path=file_path)
                
                # Calculate improvements
                peak_velocities = max(max(abs(v)) for v in results_df['velocities'])
                peak_accelerations = max(max(abs(a)) for a in results_df['accelerations'])
                
                # Calculate score (weighted sum of peak velocity and acceleration)
                score = peak_velocities + 0.5 * peak_accelerations
                
                return {
                    'leg_length': leg_length,
                    'rail_length': rail_length,
                    'score': score,
                    'peak_velocities': peak_velocities,
                    'peak_accelerations': peak_accelerations,
                    'peak_velocities_no_opt': peak_velocities_no_opt,
                    'peak_accelerations_no_opt': peak_accelerations_no_opt,
                    'pos_offset': pos_offset,
                    'rot_offset': rot_offset,
                    'feasible': hasattr(controller, 'best_feasible_solution')
                }
            except Exception as e:
                print(f"Failed with leg={leg_length:.2f}m, rail={rail_length:.2f}m: {str(e)}")
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
                result = test_configuration(leg_length, rail_length)
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
                result = test_configuration(leg_length, rail_length)
                if result and result['feasible']:
                    phase2_results.append(result)
                    print(f"Score: {result['score']:.3f}")
            
            # Combine results for this rail length
            rail_results = phase1_results + phase2_results
            if rail_results:
                all_configs.extend(rail_results)
        
        if not all_configs:
            return False, f"No feasible configurations found for {file_path}"
        
        # Find best overall configuration
        best_config = min(all_configs, key=lambda x: x['score'])
        
        print("\nBest configuration found:")
        print(f"Rail length: {best_config['rail_length']:.3f}m")
        print(f"Leg length: {best_config['leg_length']:.3f}m")
        print(f"Peak velocity: {best_config['peak_velocities']:.3f} m/s")
        print(f"Peak acceleration: {best_config['peak_accelerations']:.3f} m/s²")
        print(f"Position offset: ({best_config['pos_offset'][0]:.3f}, {best_config['pos_offset'][1]:.3f}, {best_config['pos_offset'][2]:.3f})m")
        print(f"Rotation offset: ({best_config['rot_offset'][0]:.1f}, {best_config['rot_offset'][1]:.1f}, {best_config['rot_offset'][2]:.1f})°")
        
        # Calculate improvements
        vel_improvement = (best_config['peak_velocities_no_opt'] - best_config['peak_velocities']) / best_config['peak_velocities_no_opt'] * 100
        acc_improvement = (best_config['peak_accelerations_no_opt'] - best_config['peak_accelerations']) / best_config['peak_accelerations_no_opt'] * 100
        
        print("\nImprovements with optimal configuration:")
        print(f"Velocity reduction: {vel_improvement:.1f}%")
        print(f"Acceleration reduction: {acc_improvement:.1f}%")
        
        return True, (file_path, best_config)
        
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
    
    # Create arguments for each file (note: rail_max_travel is handled inside process_file for optimization)
    process_args = [(f, zero_config) for f in file_paths]
    
    # Use half the available cores to avoid system overload
    num_processes = max(1, mp.cpu_count() // 2)
    
    print(f"\nProcessing {len(file_paths)} files using {num_processes} processes...")
    
    # Process files in parallel with progress bar
    successful = 0
    failed = 0
    optimal_configs = []
    
    with ProcessPoolExecutor(max_workers=num_processes) as executor:
        futures = [executor.submit(process_file, args) for args in process_args]
        
        for future in tqdm(as_completed(futures), total=len(futures), 
                         desc="Processing files", unit="file"):
            success, result = future.result()
            if success:
                successful += 1
                file_path, config = result
                optimal_configs.append({
                    'file': os.path.basename(file_path),
                    'rail_length': config['rail_length'],
                    'leg_length': config['leg_length'],
                    'peak_velocity': config['peak_velocities'],
                    'peak_acceleration': config['peak_accelerations'],
                    'vel_improvement': ((config['peak_velocities_no_opt'] - config['peak_velocities']) / 
                                     config['peak_velocities_no_opt'] * 100),
                    'acc_improvement': ((config['peak_accelerations_no_opt'] - config['peak_accelerations']) /
                                     config['peak_accelerations_no_opt'] * 100),
                    'score': config['score']
                })
            else:
                failed += 1
                print(f"\n{result}")
    
    # Create summary DataFrame
    if optimal_configs:
        summary_df = pd.DataFrame(optimal_configs)
        
        # Calculate overall statistics
        avg_length = summary_df['leg_length'].mean()
        std_length = summary_df['leg_length'].std()
        mode_length = summary_df['leg_length'].mode().iloc[0]
        avg_improvement = summary_df['vel_improvement'].mean()
        
        print("\nOptimization Summary:")
        print(f"Average optimal leg length: {avg_length:.3f}m (±{std_length:.3f}m)")
        print(f"Most common optimal leg length: {mode_length:.3f}m")
        print(f"Average velocity improvement: {avg_improvement:.1f}%")
        
        # Save detailed results
        summary_file = os.path.join(dir_path, "leg_length_optimization_results.xlsx")
        with pd.ExcelWriter(summary_file) as writer:
            # Overall results
            summary_df.to_excel(writer, sheet_name='All Results', index=False)
            
            # Statistics summary
            stats_df = pd.DataFrame({
                'Metric': ['Average Leg Length', 'Std Dev Leg Length', 'Mode Leg Length',
                          'Average Velocity Improvement', 'Success Rate'],
                'Value': [avg_length, std_length, mode_length,
                         avg_improvement, successful/(successful+failed)*100]
            })
            stats_df.to_excel(writer, sheet_name='Statistics', index=False)
            
            # Length distribution
            dist_df = summary_df['leg_length'].value_counts().reset_index()
            dist_df.columns = ['Leg Length', 'Count']
            dist_df.to_excel(writer, sheet_name='Length Distribution', index=False)
        
        print(f"\nDetailed results saved to: {summary_file}")
    
    # Print summary
    print(f"\nProcessing complete!")
    print(f"Successfully processed: {successful} files")
    print(f"Failed: {failed} files")

if __name__ == '__main__':
    main()
