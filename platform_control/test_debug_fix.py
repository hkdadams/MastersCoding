#!/usr/bin/env python3
"""
Test script to verify the debug file functionality works correctly
"""

import os
import pandas as pd
import numpy as np

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
                debug_file.write(f"    Peak Torque (Slider 1): {file_data.get('peak_torque_slider1_Nm', 0):.2f} Nm\n")
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
                
                # Torque analysis
                torques = [d.get('peak_torque_slider1_Nm', 0) for d in successful_files if not np.isnan(d.get('peak_torque_slider1_Nm', np.nan))]
                if torques:
                    debug_file.write("Peak Torque Statistics (Slider 1):\n")
                    debug_file.write(f"Mean Peak Torque: {np.mean(torques):.2f} Nm\n")
                    debug_file.write(f"Median Peak Torque: {np.median(torques):.2f} Nm\n")
                    debug_file.write(f"Maximum Peak Torque: {max(torques):.2f} Nm\n")
                    debug_file.write(f"Minimum Peak Torque: {min(torques):.2f} Nm\n\n")
            
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

def test_debug_functionality():
    """Test the debug file creation with sample data"""
    print("Testing debug file functionality...")
    
    # Create test data
    summary_data = [
        {
            'file_name': 'test_file_1.xlsx',
            'status': 'Success',
            'velocity_improvement_percent': 25.3,
            'acceleration_improvement_percent': 18.7,
            'peak_velocity_no_opt_ms': 0.524,
            'peak_acceleration_no_opt_ms2': 2.34,
            'peak_velocity_opt_ms': 0.391,
            'peak_acceleration_opt_ms2': 1.90,
            'peak_torque_slider1_Nm': 12.5,
            'slider_limits_hit': False,
            'violated_sliders': '',
            'processing_time_seconds': 45.2
        },
        {
            'file_name': 'test_file_2.xlsx',
            'status': 'Success',
            'velocity_improvement_percent': 31.1,
            'acceleration_improvement_percent': 22.8,
            'peak_velocity_no_opt_ms': 0.487,
            'peak_acceleration_no_opt_ms2': 2.15,
            'peak_velocity_opt_ms': 0.335,
            'peak_acceleration_opt_ms2': 1.66,
            'peak_torque_slider1_Nm': 15.8,
            'slider_limits_hit': True,
            'violated_sliders': 'Slider 1, Slider 3',
            'processing_time_seconds': 52.7
        },
        {
            'file_name': 'test_file_3.xlsx',
            'status': 'Failed',
            'velocity_improvement_percent': np.nan,
            'acceleration_improvement_percent': np.nan,
            'peak_velocity_no_opt_ms': np.nan,
            'peak_acceleration_no_opt_ms2': np.nan,
            'peak_velocity_opt_ms': np.nan,
            'peak_acceleration_opt_ms2': np.nan,
            'peak_torque_slider1_Nm': np.nan,
            'slider_limits_hit': False,
            'violated_sliders': '',
            'processing_time_seconds': 12.3
        }
    ]
    
    # Test statistics
    successful_data = [d for d in summary_data if d['status'] == 'Success']
    velocity_improvements = [d['velocity_improvement_percent'] for d in successful_data]
    acceleration_improvements = [d['acceleration_improvement_percent'] for d in successful_data]
    
    vel_stats = {
        'mean': np.mean(velocity_improvements),
        'median': np.median(velocity_improvements),
        'std': np.std(velocity_improvements),
        'min': np.min(velocity_improvements),
        'max': np.max(velocity_improvements)
    }
    
    acc_stats = {
        'mean': np.mean(acceleration_improvements),
        'median': np.median(acceleration_improvements),
        'std': np.std(acceleration_improvements),
        'min': np.min(acceleration_improvements),
        'max': np.max(acceleration_improvements)
    }
    
    # Test slider violations
    files_with_slider_violations = [
        {
            'file_name': 'test_file_2.xlsx',
            'violated_sliders': ['Slider 1', 'Slider 3'],
            'details': {
                'Slider 1': 'max(0.5023m)',
                'Slider 3': 'min(-0.0015m)'
            }
        }
    ]
    
    # Create output directory
    output_dir = os.path.join(os.getcwd(), "test_debug_output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Test the function
    total_time = 110.2  # seconds
    successful = 2
    failed = 1
    
    success = create_batch_debug_files(
        summary_data, vel_stats, acc_stats, files_with_slider_violations,
        output_dir, total_time, successful, failed
    )
    
    if success:
        print("✓ Debug file functionality test PASSED!")
        print(f"Check the files in: {output_dir}")
    else:
        print("✗ Debug file functionality test FAILED!")

if __name__ == '__main__':
    test_debug_functionality()
