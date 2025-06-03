##### WARNING: THIS SCRIPT IS DESIGNED TO RUN IN A MULTIPROCESSING ENVIRONMENT #####
##### IT CONTAINS AN OPTIMIZATION ALGORITHM FOR STARTING POSITION, DESIGNED TO BE RELIABLE BUT NOT FAST #####
##### THIS CAN TAKE A SIGNIFICANT AMOUNT OF TIME TO RUN UNLESS YOUR COMPUTER HAS A LARGE NUMBER OF CORES DO NOT RUN!#####

import os
import pandas as pd
import numpy as np
import gc
from tqdm import tqdm
from platform_controllerMP import PlatformController

def cleanup_memory():
    """Force garbage collection to free memory between files"""
    gc.collect()

def create_batch_debug_files(summary_data, vel_stats, acc_stats, files_with_slider_violations, 
                           output_dir, total_time, successful, failed):
    """Create comprehensive debug files with batch analysis data"""
    try:
        # Create main debug file
        debug_file_path = os.path.join(output_dir, "batch_analysis_debug.txt")
        
        with open(debug_file_path, 'w', encoding='utf-8') as debug_file:
            # Header
            debug_file.write("BATCH PROCESSING ANALYSIS RESULTS\n")
            debug_file.write("=" * 50 + "\n\n")
            
            # Processing Summary
            debug_file.write("Processing Summary:\n")
            debug_file.write(f"Total Files Processed: {successful + failed}\n")
            debug_file.write(f"Successfully Processed: {successful}\n")
            debug_file.write(f"Failed: {failed}\n")
            debug_file.write(f"Total Processing Time: {total_time/60:.1f} minutes\n")
            debug_file.write(f"Average Time per File: {total_time/max(successful + failed, 1):.1f} seconds\n\n")
            
            # Velocity Improvement Analysis
            if vel_stats:
                debug_file.write("Velocity Reduction Analysis:\n")
                debug_file.write(f"Mean Reduction: {vel_stats['mean']:.2f}%\n")
                debug_file.write(f"Median Reduction: {vel_stats['median']:.2f}%\n")
                debug_file.write(f"Standard Deviation: {vel_stats['std']:.2f}%\n")
                debug_file.write(f"Minimum Reduction: {vel_stats['min']:.2f}%\n")
                debug_file.write(f"Maximum Reduction: {vel_stats['max']:.2f}%\n\n")
            
            # Acceleration Improvement Analysis
            if acc_stats:
                debug_file.write("Acceleration Reduction Analysis:\n")
                debug_file.write(f"Mean Reduction: {acc_stats['mean']:.2f}%\n")
                debug_file.write(f"Median Reduction: {acc_stats['median']:.2f}%\n")
                debug_file.write(f"Standard Deviation: {acc_stats['std']:.2f}%\n")
                debug_file.write(f"Minimum Reduction: {acc_stats['min']:.2f}%\n")
                debug_file.write(f"Maximum Reduction: {acc_stats['max']:.2f}%\n\n")
            
            # Slider Limit Violation Analysis
            debug_file.write("Slider Limit Violation Analysis:\n")
            total_successful = len([d for d in summary_data if d['status'] == 'Success'])
            num_violations = len(files_with_slider_violations)
            violation_percentage = (num_violations / total_successful) * 100 if total_successful > 0 else 0
            
            debug_file.write(f"Files with Slider Violations: {num_violations}\n")
            debug_file.write(f"Total Successful Files: {total_successful}\n")
            debug_file.write(f"Violation Percentage: {violation_percentage:.1f}%\n\n")
            
            if files_with_slider_violations:
                debug_file.write("Detailed Violation List:\n")
                for violation in files_with_slider_violations:
                    debug_file.write(f"File: {violation['file_name']}\n")
                    debug_file.write(f"Violated Sliders: {', '.join(violation['violated_sliders'])}\n")
                    for slider, details in violation['details'].items():
                        debug_file.write(f"  {slider}: {details}\n")
                    debug_file.write("\n")
            else:
                debug_file.write("No slider limit violations detected!\n\n")
            
            # Per-File Analysis Summary
            debug_file.write("=" * 50 + "\n")
            debug_file.write("PER-FILE ANALYSIS SUMMARY\n")
            debug_file.write("=" * 50 + "\n\n")
            
            successful_files = [d for d in summary_data if d['status'] == 'Success']
            failed_files = [d for d in summary_data if d['status'] == 'Failed']
              # Sort by velocity improvement for ranking
            successful_files.sort(key=lambda x: x.get('velocity_improvement_percent', 0), reverse=True)
            
            debug_file.write("Successful Files (Ranked by Velocity Improvement):\n")
            debug_file.write("-" * 50 + "\n")
            for i, file_data in enumerate(successful_files, 1):
                debug_file.write(f"{i:2d}. {file_data['file_name']}\n")
                debug_file.write(f"    Velocity Reduction: {file_data.get('velocity_improvement_percent', 0):.2f}%\n")
                debug_file.write(f"    Acceleration Reduction: {file_data.get('acceleration_improvement_percent', 0):.2f}%\n")
                debug_file.write(f"    Processing Time: {file_data.get('processing_time_seconds', 0):.1f}s\n")
                if file_data.get('slider_limits_hit', False):
                    debug_file.write(f"    WARNING - Slider Violations: {file_data.get('violated_sliders', '')}\n")
                debug_file.write("\n")
            
            if failed_files:
                debug_file.write("Failed Files:\n")
                debug_file.write("-" * 50 + "\n")
                for file_data in failed_files:
                    debug_file.write(f"• {file_data['file_name']}\n")
                    debug_file.write(f"  Processing Time: {file_data.get('processing_time_seconds', 0):.1f}s\n\n")
            
            # Performance Analysis
            debug_file.write("=" * 50 + "\n")
            debug_file.write("PERFORMANCE ANALYSIS\n")
            debug_file.write("=" * 50 + "\n\n")
            
            if successful_files:
                # Processing time analysis
                processing_times = [d.get('processing_time_seconds', 0) for d in successful_files if d.get('processing_time_seconds', 0) > 0]
                if processing_times:
                    debug_file.write("Processing Time Statistics:\n")
                    debug_file.write(f"Mean Processing Time: {np.mean(processing_times):.1f} seconds\n")
                    debug_file.write(f"Median Processing Time: {np.median(processing_times):.1f} seconds\n")
                    debug_file.write(f"Fastest File: {min(processing_times):.1f} seconds\n")
                    debug_file.write(f"Slowest File: {max(processing_times):.1f} seconds\n\n")
            
            debug_file.write("=" * 50 + "\n")
            debug_file.write(f"Analysis completed at: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # Create additional detailed CSV for further analysis
        csv_file_path = os.path.join(output_dir, "batch_analysis_detailed.csv")
        detailed_df = pd.DataFrame(summary_data)
        detailed_df.to_csv(csv_file_path, index=False)
        
        print(f"\nDebug files created:")
        print(f"  - Comprehensive analysis: {debug_file_path}")
        print(f"  - Detailed CSV data: {csv_file_path}")
        
        return True
        
    except Exception as e:
        print(f"\nError creating debug files: {e}")
        return False

def process_file(args):
    """Process a single Excel file with pre-processed position and orientation data"""
    file_path, leg_length, rail_max_travel, slider_base_position, platform_side, platform_radius, enable_logging = args
    error_details = {}
    start_time = pd.Timestamp.now()
    df = None
    trajectory_df = None
    
    try:
        # Create output directory
        output_dir = os.path.join(os.path.dirname(file_path), "platform_outputs")
        os.makedirs(output_dir, exist_ok=True)
        
        # Set up logging
        input_file_name = os.path.basename(file_path).replace('.xlsx', '')
        log_file_path = os.path.join(output_dir, f"debug_log_{input_file_name}.txt")

        # Read the Position_Orientation sheet directly
        df = pd.read_excel(file_path, sheet_name='Position_Orientation')
        
        # Check for required columns
        required_columns = ['t(ms)', 'X(m)', 'Y(m)', 'Z(m)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)']
        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            raise ValueError(f"Missing required columns in Position_Orientation sheet: {', '.join(missing_columns)}")
        
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
        trajectory_df = df.rename(columns=column_map)
          # Convert time from milliseconds to seconds
        trajectory_df['time'] = trajectory_df['time'] / 1000.0
        
        # Create controller and process
        controller = PlatformController(leg_length, rail_max_travel, slider_base_position, log_file_path, platform_side=platform_side, platform_radius=platform_radius, log_attempts=enable_logging)
        
        # Process without optimization first
        results_df_no_opt = controller.process_trajectory(
            trajectory_df, 
            file_path=file_path,
            apply_optimization=False
        )
        
        if results_df_no_opt is None:
            raise ValueError("Failed to process trajectory without optimization")
        
        # Get initial metrics
        peak_velocities_no_opt = max(max(abs(v)) for v in results_df_no_opt['velocities'])
        peak_accelerations_no_opt = max(max(abs(a)) for a in results_df_no_opt['accelerations'])
        
        print(f"\nProcessing {os.path.basename(file_path)}")
        print(f"Initial peak velocity: {peak_velocities_no_opt:.3f} m/s")
        print(f"Initial peak acceleration: {peak_accelerations_no_opt:.3f} m/s²")
        
        # Optimize and process
        pos_offset, rot_offset = controller.optimize_offsets(
            trajectory_df=trajectory_df,
            log_file_path=log_file_path
        )
        
        if pos_offset is None or rot_offset is None:
            raise ValueError("No feasible solution found during optimization")
        
        results_df = controller.process_trajectory(
            trajectory_df, 
            file_path=file_path
        )
        
        if results_df is None:
            raise ValueError("Failed to process optimized trajectory")          # Calculate improvements
        peak_velocities = max(max(abs(v)) for v in results_df['velocities'])
        peak_accelerations = max(max(abs(a)) for a in results_df['accelerations'])
        
        vel_improvement = (peak_velocities_no_opt - peak_velocities) / peak_velocities_no_opt * 100
        acc_improvement = (peak_accelerations_no_opt - peak_accelerations) / peak_accelerations_no_opt * 100
        
        # Check for slider limit violations
        slider_limits_hit = False
        violated_sliders = []
        slider_limit_details = {}
        
        if 'Slider Position 1 (m)' in results_df.columns and 'Slider Position 2 (m)' in results_df.columns and 'Slider Position 3 (m)' in results_df.columns:
            min_allowed = slider_base_position
            max_allowed = slider_base_position + rail_max_travel
            
            for i, col in enumerate(['Slider Position 1 (m)', 'Slider Position 2 (m)', 'Slider Position 3 (m)'], 1):
                slider_positions = results_df[col]
                min_pos = slider_positions.min()
                max_pos = slider_positions.max()
                
                # Check if any position hits the limits (with small tolerance)
                hits_min = min_pos <= (min_allowed - 1e-6)
                hits_max = max_pos >= (max_allowed + 1e-6)
                
                if hits_min or hits_max:
                    slider_limits_hit = True
                    violated_sliders.append(f"Slider {i}")
                    limit_type = []
                    if hits_min:
                        limit_type.append(f"min({min_pos:.4f}m)")
                    if hits_max:
                        limit_type.append(f"max({max_pos:.4f}m)")
                    slider_limit_details[f"Slider {i}"] = ", ".join(limit_type)
        
        processing_time = (pd.Timestamp.now() - start_time).total_seconds()
        print(f"Processing complete in {processing_time:.1f} seconds")
        print(f"Velocity reduction: {vel_improvement:.1f}%")
        print(f"Acceleration reduction: {acc_improvement:.1f}%")
        
        if slider_limits_hit:
            print(f"⚠ SLIDER LIMIT VIOLATIONS: {', '.join(violated_sliders)}")
            for slider, details in slider_limit_details.items():
                print(f"  {slider}: {details}")
        else:
            print("✓ No slider limit violations detected")
        
        # Return comprehensive results
        return True, file_path, None, {
            'vel_improvement': vel_improvement,
            'acc_improvement': acc_improvement,
            'peak_velocities_no_opt': peak_velocities_no_opt,
            'peak_accelerations_no_opt': peak_accelerations_no_opt,
            'peak_velocities_opt': peak_velocities,
            'peak_accelerations_opt': peak_accelerations,
            'slider_limits_hit': slider_limits_hit,
            'violated_sliders': violated_sliders,
            'slider_limit_details': slider_limit_details,            'processing_time': processing_time
        }
        
    except Exception as e:
        error_details['error'] = str(e)
        
        # Log error
        try:
            error_log_path = os.path.join(output_dir, "processing_errors.log")
            with open(error_log_path, 'a') as error_log:
                error_log.write(f"\n{'='*50}\n")
                error_log.write(f"File: {file_path}\n")
                error_log.write(f"Time: {pd.Timestamp.now()}\n")
                error_log.write(f"Duration: {(pd.Timestamp.now() - start_time).total_seconds():.1f}s\n")
                error_log.write(f"Error: {str(e)}\n")
                error_log.write(f"{'='*50}\n")
        except Exception as log_error:
            print(f"Failed to write to error log: {str(log_error)}")
            
        return False, file_path, error_details, {
            'vel_improvement': np.nan,
            'acc_improvement': np.nan,
            'peak_velocities_no_opt': np.nan,
            'peak_accelerations_no_opt': np.nan,
            'peak_velocities_opt': np.nan,
            'peak_accelerations_opt': np.nan,
            'slider_limits_hit': False,
            'violated_sliders': [],
            'slider_limit_details': {},
            'processing_time': (pd.Timestamp.now() - start_time).total_seconds()
        }
        
    finally:
        # Clean up DataFrames
        if df is not None:
            del df
        if trajectory_df is not None:
            del trajectory_df
        cleanup_memory()

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
    
    # Check for existing output files first
    if not check_existing_outputs(file_paths):
        return
    
    # Get geometric parameters
    print(f"\n{'='*60}")
    print("PLATFORM CONFIGURATION")
    print(f"{'='*60}")
    
    leg_length = float(input("\nEnter leg length in meters (default 0.5): ") or "0.5")
    rail_max_travel = float(input("Enter maximum rail travel distance in meters (default 1.0): ") or "1.0")
    
    # Platform size configuration
    print(f"\n{'-'*50}")
    print("PLATFORM SIZE CONFIGURATION")
    print(f"{'-'*50}")
    print("You can specify the platform size using either:")
    print("  1. Side length (length of each edge of the triangular platform)")
    print("  2. Radius (distance from center to leg attachment points)")
    print("")
    print("Choose your preferred input method:")
    print("  [1] Side length (default)")
    print("  [2] Radius")
    
    choice = input("\nEnter choice (1 or 2, default 1): ").strip() or "1"
    
    platform_side = None
    platform_radius = None
    
    if choice == "2":
        print("\nPlatform radius examples:")
        print("  • 0.2565m: Default radius (side ≈ 0.44445926m)")
        print("  • 0.173m: Smaller radius (side ≈ 0.3m)")
        print("  • 0.346m: Larger radius (side ≈ 0.6m)")
        
        platform_radius = float(input(f"\nEnter platform radius in meters (default 0.2565): ") or "0.2565")
        platform_side_calc = platform_radius * (3**0.5)  # Calculate side for display
        print(f"✓ Platform side length will be: {platform_side_calc:.3f}m")
    else:
        print("\nPlatform side length examples:")
        print("  • 0.44445926m: Default side length (radius ≈ 0.256m)")
        print("  • 0.3m: Smaller platform (radius ≈ 0.173m)")
        print("  • 0.6m: Larger platform (radius ≈ 0.346m)")
        
        platform_side = float(input(f"\nEnter platform side length in meters (default 0.44445926): ") or "0.44445926")
        platform_radius_calc = platform_side / (3**0.5)  # Calculate radius for display
        print(f"✓ Platform radius will be: {platform_radius_calc:.3f}m")
    
    # Improved slider position offset explanation
    print(f"\n{'-'*50}")
    print("SLIDER POSITION CONFIGURATION")
    print(f"{'-'*50}")
    print("The slider base position sets where the slider's 'zero' position is located")
    print("in global coordinates. This defines the starting point of the slider's travel range.")
    print("Examples:")
    print("  • 0.0m: Sliders operate from 0.0m to rail_travel distance")
    print("  • 0.1m: Sliders operate from 0.1m to (0.1m + rail_travel)")
    print("  • -0.05m: Sliders operate from -0.05m to (-0.05m + rail_travel)")
    
    slider_base_position = float(input(f"\nEnter slider base position in meters (default 0.25406848): ") or "0.25406848")
    
    # Show the calculated operating range
    slider_min_limit = slider_base_position
    slider_max_limit = slider_base_position + rail_max_travel
    print(f"\n✓ Sliders will operate in range: {slider_min_limit:.3f}m to {slider_max_limit:.3f}m")
      # Add logging option
    print(f"\n{'-'*50}")
    print("LOGGING OPTIONS")
    print(f"{'-'*50}")
    enable_logging = input("Enable detailed optimization logging? (y/N): ").lower().startswith('y')
    
    # Create arguments for each file
    process_args = [(f, leg_length, rail_max_travel, slider_base_position, platform_side, platform_radius, enable_logging) for f in file_paths]
    
    print(f"\nProcessing {len(file_paths)} files sequentially...")
    start_time = pd.Timestamp.now()
    
    # Process files one at a time with progress bar
    successful = 0
    failed = 0
    failed_files = []
    summary_data = [] # Added
    
    # Track metrics for statistics
    velocity_improvements = []
    acceleration_improvements = []
    files_with_slider_violations = []
    
    try:
        # Create a progress bar for sequential processing
        for i, args in enumerate(tqdm(process_args, desc="Processing files", unit="file")):
            success, file_path, error_details, metrics = process_file(args) # Modified
            file_name = os.path.basename(file_path)
            
            if success:
                successful += 1
                
                # Extract metrics
                vel_improvement = metrics['vel_improvement']
                acc_improvement = metrics['acc_improvement']
                slider_limits_hit = metrics['slider_limits_hit']
                violated_sliders = metrics['violated_sliders']
                
                # Store for statistics
                velocity_improvements.append(vel_improvement)
                acceleration_improvements.append(acc_improvement)
                
                if slider_limits_hit:
                    files_with_slider_violations.append({
                        'file_name': file_name,
                        'violated_sliders': violated_sliders,
                        'details': metrics['slider_limit_details']                    })
                
                summary_data.append({
                    'file_name': file_name,
                    'status': 'Success',
                    'velocity_improvement_percent': vel_improvement,
                    'acceleration_improvement_percent': acc_improvement,
                    'peak_velocity_no_opt_ms': metrics['peak_velocities_no_opt'],
                    'peak_acceleration_no_opt_ms2': metrics['peak_accelerations_no_opt'],
                    'peak_velocity_opt_ms': metrics['peak_velocities_opt'],
                    'peak_acceleration_opt_ms2': metrics['peak_accelerations_opt'],
                    'slider_limits_hit': slider_limits_hit,
                    'violated_sliders': ', '.join(violated_sliders) if violated_sliders else '',
                    'processing_time_seconds': metrics['processing_time']                })
            else:
                failed += 1
                failed_files.append((file_path, error_details))
                
                summary_data.append({
                    'file_name': file_name,
                    'status': 'Failed',
                    'velocity_improvement_percent': np.nan,
                    'acceleration_improvement_percent': np.nan,
                    'peak_velocity_no_opt_ms': np.nan,
                    'peak_acceleration_no_opt_ms2': np.nan,
                    'peak_velocity_opt_ms': np.nan,
                    'peak_acceleration_opt_ms2': np.nan,
                    'slider_limits_hit': False,
                    'violated_sliders': '',
                    'processing_time_seconds': metrics['processing_time'] if 'processing_time' in metrics else np.nan
                })
                print(f"\nError processing {file_name}:")
                for error_type, error_msg in error_details.items():
                    print(f"  {error_type}: {error_msg}")
            
            # Show estimated time remaining
            elapsed_time = (pd.Timestamp.now() - start_time).total_seconds()
            files_remaining = len(file_paths) - (i + 1)
            if i > 0:  # Only show estimate after first file
                avg_time_per_file = elapsed_time / (i + 1)
                est_remaining = avg_time_per_file * files_remaining
                print(f"\nEstimated time remaining: {est_remaining/60:.1f} minutes")
    
    except KeyboardInterrupt:
        print("\n\nProcessing interrupted by user!")
        cleanup_memory()
    
    finally:
        # Calculate total processing time
        total_time = (pd.Timestamp.now() - start_time).total_seconds()
        
        # Initialize statistics variables
        vel_stats = None
        acc_stats = None
        output_dir = os.path.join(dir_path, "platform_outputs")
        os.makedirs(output_dir, exist_ok=True)
          # Calculate and display improvement statistics
        if velocity_improvements and acceleration_improvements:
            print(f"\n{'='*60}")
            print("IMPROVEMENT STATISTICS")
            print(f"{'='*60}")
              # Filter out negative percentage reductions (only include feasible improvements)
            feasible_velocity_improvements = [v for v in velocity_improvements if v > 0]
            feasible_acceleration_improvements = [a for a in acceleration_improvements if a > 0]
            
            print(f"\nFiltered {len(velocity_improvements) - len(feasible_velocity_improvements)} infeasible velocity improvements")
            print(f"Filtered {len(acceleration_improvements) - len(feasible_acceleration_improvements)} infeasible acceleration improvements")
              # Velocity improvement statistics (feasible improvements only)
            if feasible_velocity_improvements:
                vel_stats = {
                    'mean': np.mean(feasible_velocity_improvements),
                    'median': np.median(feasible_velocity_improvements),
                    'std': np.std(feasible_velocity_improvements),
                    'min': np.min(feasible_velocity_improvements),
                    'max': np.max(feasible_velocity_improvements)
                }
                
                print(f"\nVelocity Reduction Statistics (Feasible Improvements Only):")
                print(f"  Count:    {len(feasible_velocity_improvements)} out of {len(velocity_improvements)} files")
                print(f"  Mean:     {vel_stats['mean']:.2f}%")
                print(f"  Median:   {vel_stats['median']:.2f}%")
                print(f"  Std Dev:  {vel_stats['std']:.2f}%")
                print(f"  Min:      {vel_stats['min']:.2f}%")
                print(f"  Max:      {vel_stats['max']:.2f}%")
            else:
                vel_stats = None
                print(f"\nVelocity Reduction Statistics: No feasible improvements found")
              # Acceleration improvement statistics (feasible improvements only)
            if feasible_acceleration_improvements:
                acc_stats = {
                    'mean': np.mean(feasible_acceleration_improvements),
                    'median': np.median(feasible_acceleration_improvements),
                    'std': np.std(feasible_acceleration_improvements),
                    'min': np.min(feasible_acceleration_improvements),
                    'max': np.max(feasible_acceleration_improvements)
                }
                
                print(f"\nAcceleration Reduction Statistics (Feasible Improvements Only):")
                print(f"  Count:    {len(feasible_acceleration_improvements)} out of {len(acceleration_improvements)} files")
                print(f"  Mean:     {acc_stats['mean']:.2f}%")
                print(f"  Median:   {acc_stats['median']:.2f}%")
                print(f"  Std Dev:  {acc_stats['std']:.2f}%")
                print(f"  Min:      {acc_stats['min']:.2f}%")
                print(f"  Max:      {acc_stats['max']:.2f}%")
            else:
                acc_stats = None
                print(f"\nAcceleration Reduction Statistics: No feasible improvements found")
              # Slider limit violation statistics
            print(f"\n{'='*60}")
            print("SLIDER LIMIT VIOLATION ANALYSIS")
            print(f"{'='*60}")
            
            total_successful = len(velocity_improvements)
            num_violations = len(files_with_slider_violations)
            violation_percentage = (num_violations / total_successful) * 100 if total_successful > 0 else 0
            
            print(f"\nFiles hitting slider limits: {num_violations} out of {total_successful} successful files ({violation_percentage:.1f}%)")
            
            if files_with_slider_violations:
                print(f"\nFiles with slider limit violations:")
                for violation in files_with_slider_violations:
                    print(f"  - {violation['file_name']}: {', '.join(violation['violated_sliders'])}")
                    for slider, details in violation['details'].items():
                        print(f"    {slider}: {details}")
            else:
                print("✓ No files hit slider limits!")
            
            # Create comprehensive debug files (moved here where variables are in scope)
            try:
                create_debug_success = create_batch_debug_files(summary_data, vel_stats, acc_stats, files_with_slider_violations, 
                               output_dir, total_time, successful, failed)
                
                if create_debug_success:
                    print(f"\nComprehensive debug files created in: {output_dir}")
                else:
                    print(f"\nFailed to create debug files")
            except Exception as e:
                print(f"\nError during debug file creation: {e}")
        
        # Create summary DataFrame and save to Excel
        if summary_data: # Added
            summary_df = pd.DataFrame(summary_data) # Added
            summary_output_dir = os.path.join(dir_path, "platform_outputs") # Added
            os.makedirs(summary_output_dir, exist_ok=True) # Added
            summary_excel_path = os.path.join(summary_output_dir, "batch_processing_summary_results.xlsx") # Added
            try: # Added
                # Create a comprehensive Excel file with multiple sheets
                with pd.ExcelWriter(summary_excel_path, engine='openpyxl') as writer:
                    # Main summary sheet
                    summary_df.to_excel(writer, index=False, sheet_name="File_Results")
                      # Statistics summary sheet
                    if velocity_improvements and acceleration_improvements:
                        stats_data = []
                          # Velocity stats (only if feasible improvements exist)
                        if vel_stats is not None:
                            vel_stats_row = {
                                'Metric': 'Velocity Reduction (% - Feasible Only)',
                                'Count': len([v for v in velocity_improvements if v > 0]),
                                'Total_Files': len(velocity_improvements),
                                'Mean': vel_stats['mean'],
                                'Median': vel_stats['median'],
                                'Std_Dev': vel_stats['std'],
                                'Min': vel_stats['min'],
                                'Max': vel_stats['max']
                            }
                            stats_data.append(vel_stats_row)
                        else:
                            vel_stats_row = {
                                'Metric': 'Velocity Reduction (% - Feasible Only)',
                                'Count': 0,
                                'Total_Files': len(velocity_improvements),
                                'Mean': 'No feasible improvements',
                                'Median': 'No feasible improvements',
                                'Std_Dev': 'No feasible improvements',
                                'Min': 'No feasible improvements',
                                'Max': 'No feasible improvements'
                            }
                            stats_data.append(vel_stats_row)
                        
                        # Acceleration stats (only if feasible improvements exist)
                        if acc_stats is not None:
                            acc_stats_row = {
                                'Metric': 'Acceleration Reduction (% - Feasible Only)',
                                'Count': len([a for a in acceleration_improvements if a > 0]),
                                'Total_Files': len(acceleration_improvements),
                                'Mean': acc_stats['mean'],
                                'Median': acc_stats['median'],
                                'Std_Dev': acc_stats['std'],
                                'Min': acc_stats['min'],
                                'Max': acc_stats['max']
                            }
                            stats_data.append(acc_stats_row)
                        else:
                            acc_stats_row = {
                                'Metric': 'Acceleration Reduction (% - Feasible Only)',
                                'Count': 0,
                                'Total_Files': len(acceleration_improvements),
                                'Mean': 'No feasible improvements',
                                'Median': 'No feasible improvements',
                                'Std_Dev': 'No feasible improvements',
                                'Min': 'No feasible improvements',
                                'Max': 'No feasible improvements'
                            }
                            stats_data.append(acc_stats_row)
                        
                        stats_df = pd.DataFrame(stats_data)
                        stats_df.to_excel(writer, index=False, sheet_name="Statistics")
                        
                        # Slider violations sheet
                        if files_with_slider_violations:
                            violation_data = []
                            for violation in files_with_slider_violations:
                                violation_data.append({
                                    'File_Name': violation['file_name'],
                                    'Violated_Sliders': ', '.join(violation['violated_sliders']),
                                    'Details': '; '.join([f"{k}: {v}" for k, v in violation['details'].items()])
                                })
                            violation_df = pd.DataFrame(violation_data)
                            violation_df.to_excel(writer, index=False, sheet_name="Slider_Violations")
                    
                print(f"\nBatch processing summary saved to: {summary_excel_path}")
            except Exception as e:
                print(f"\nError saving summary Excel: {e}")
        
        # Create comprehensive debug files (moved to end of summary section)

        # Print summary        print(f"\n{'='*60}")
        print("BATCH PROCESSING SUMMARY")
        print(f"{'='*60}")
        print(f"Total processing time: {total_time/60:.1f} minutes")
        print(f"Successfully processed: {successful} files")
        print(f"Failed: {failed} files")
        
        if failed > 0:
            print("\nFailed files summary:")
            for file_path, error_details in failed_files:
                print(f"\n{os.path.basename(file_path)}:")
                for error_type, error_msg in error_details.items():
                    print(f"  {error_type}: {error_msg}")
            
            # Point to the error log
            output_dir = os.path.join(os.path.dirname(file_paths[0]), "platform_outputs")
            print(f"\nDetailed error logs have been written to: {os.path.join(output_dir, 'processing_errors.log')}")

if __name__ == '__main__':
    main()
