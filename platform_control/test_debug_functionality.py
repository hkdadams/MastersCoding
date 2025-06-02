#!/usr/bin/env python3
"""
Test script to verify the debug file creation functionality
"""

import os
import numpy as np
import pandas as pd
from Batch_process_motions import create_batch_debug_files

def test_debug_file_creation():
    """Test the debug file creation with sample data"""
    
    # Create test data
    summary_data = [
        {
            'file_name': 'Rugby_test_1.xlsx',
            'status': 'Success',
            'velocity_improvement_percent': 15.3,
            'acceleration_improvement_percent': 22.7,
            'peak_velocity_no_opt_ms': 5.2,
            'peak_acceleration_no_opt_ms2': 85.4,
            'peak_velocity_opt_ms': 4.4,
            'peak_acceleration_opt_ms2': 66.1,
            'peak_torque_slider1_Nm': 12.5,
            'slider_limits_hit': False,
            'violated_sliders': '',
            'processing_time_seconds': 45.2
        },
        {
            'file_name': 'Rugby_test_2.xlsx',
            'status': 'Success',
            'velocity_improvement_percent': 8.9,
            'acceleration_improvement_percent': 18.1,
            'peak_velocity_no_opt_ms': 6.1,
            'peak_acceleration_no_opt_ms2': 92.3,
            'peak_velocity_opt_ms': 5.6,
            'peak_acceleration_opt_ms2': 75.6,
            'peak_torque_slider1_Nm': 15.8,
            'slider_limits_hit': True,
            'violated_sliders': 'Slider 1, Slider 3',
            'processing_time_seconds': 52.1
        },
        {
            'file_name': 'Rugby_test_3.xlsx',
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
            'processing_time_seconds': 8.3
        }
    ]
    
    # Create test statistics
    vel_stats = {
        'mean': 12.1,
        'median': 12.1,
        'std': 4.5,
        'min': 8.9,
        'max': 15.3
    }
    
    acc_stats = {
        'mean': 20.4,
        'median': 20.4,
        'std': 3.3,
        'min': 18.1,
        'max': 22.7
    }
    
    # Create test slider violations
    files_with_slider_violations = [
        {
            'file_name': 'Rugby_test_2.xlsx',
            'violated_sliders': ['Slider 1', 'Slider 3'],
            'details': {
                'Slider 1': 'min(0.0001m)',
                'Slider 3': 'max(0.4999m)'
            }
        }
    ]
    
    # Create test output directory
    test_output_dir = os.path.join(os.path.dirname(__file__), "test_debug_output")
    os.makedirs(test_output_dir, exist_ok=True)
    
    print("Testing debug file creation...")
    
    # Test the debug file creation
    success = create_batch_debug_files(
        summary_data=summary_data,
        vel_stats=vel_stats,
        acc_stats=acc_stats,
        files_with_slider_violations=files_with_slider_violations,
        output_dir=test_output_dir,
        total_time=180.5,  # 3 minutes total
        successful=2,
        failed=1
    )
    
    if success:
        print("✓ Debug file creation test PASSED")
        
        # Check if files were created
        debug_file = os.path.join(test_output_dir, "batch_analysis_debug.txt")
        csv_file = os.path.join(test_output_dir, "batch_analysis_detailed.csv")
        
        if os.path.exists(debug_file):
            print(f"✓ Debug text file created: {debug_file}")
            print(f"File size: {os.path.getsize(debug_file)} bytes")
        else:
            print("✗ Debug text file not found")
            
        if os.path.exists(csv_file):
            print(f"✓ Debug CSV file created: {csv_file}")
            print(f"File size: {os.path.getsize(csv_file)} bytes")
        else:
            print("✗ Debug CSV file not found")
            
        # Show a sample of the debug file content
        if os.path.exists(debug_file):
            print("\n--- Sample Debug File Content ---")
            with open(debug_file, 'r') as f:
                lines = f.readlines()[:20]  # First 20 lines
                for line in lines:
                    print(line.rstrip())
            print("--- End Sample ---\n")
    else:
        print("✗ Debug file creation test FAILED")
    
    return success

if __name__ == "__main__":
    test_debug_file_creation()
