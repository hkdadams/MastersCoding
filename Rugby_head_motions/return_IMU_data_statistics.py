'''
Script to analyze rugby head motion data from Excel files.
Calculates peak linear and rotational accelerations and provides summary statistics.

Usage:
    python analyze_rugby_head_motions.py file1.xlsx file2.xlsx ...
    
    If no files are provided, the script will prompt for file paths interactively.
'''
import pandas as pd
import numpy as np
import os
import glob
import sys
import argparse

def process_file(file_path):
    """
    Processes a single rugby head motion Excel file.
    Calculates peak linear and rotational accelerations.

    Args:
        file_path (str): The absolute path to the Excel file.

    Returns:
        tuple: (peak_linear_acceleration, peak_rotational_acceleration) or (None, None) if processing fails.
    """
    try:
        df = pd.read_excel(file_path)
    except Exception as e:
        print(f"Error reading {os.path.basename(file_path)}: {e}")
        return None, None
        
    required_cols = ['LinAccX', 'LinAccY', 'LinAccZ', 'LinAccRes', 
                   'RotVelX', 'RotVelY', 'RotVelZ', 'RotVelRes', 
                   'RotAccX', 'RotAccY', 'RotAccZ', 'RotAccRes', 't(ms)']
    
    if not all(col in df.columns for col in required_cols):
        missing_cols = [col for col in required_cols if col not in df.columns]
        print(f"Skipping {os.path.basename(file_path)}: Missing columns: {missing_cols}")
        return None, None

    # Ensure columns are numeric and handle potential errors by coercing to NaN
    for col in required_cols:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # Drop rows where any required column has NaN and reset index
    df = df.dropna(subset=required_cols).reset_index(drop=True)

    if len(df) < 2:  # Need at least two valid data points
        print(f"Skipping {os.path.basename(file_path)}: Insufficient valid data rows ({len(df)}) after cleaning.")
        return None, None
    
    # Since we already have the linear acceleration and rotational acceleration data,
    # we can directly extract the peak values from the resultant columns
    peak_lin_acc = df['LinAccRes'].max()
    peak_rot_acc = df['RotAccRes'].max()
    
    if pd.isna(peak_lin_acc) or pd.isna(peak_rot_acc):
        print(f"  Warning: Peak acceleration calculation resulted in NaN for {os.path.basename(file_path)}. ")
        print(f"           This might be due to insufficient data variation (e.g., constant values) or too few data points.")
        return None, None

    return peak_lin_acc, peak_rot_acc

def get_files_interactively():
    """Prompt the user to input file paths or folder paths interactively."""
    excel_files = []
    print("Enter the file paths or folder paths (one per line). Enter an empty line when done:")

    while True:
        input_path = input("> ").strip()
        if not input_path:
            break

        # Handle paths with or without quotes
        input_path = input_path.strip('"\'')

        if os.path.exists(input_path):
            if os.path.isdir(input_path):
                # If it's a folder, retrieve all Excel files from the folder
                excel_files.extend(get_files_from_folder(input_path))
            elif input_path.lower().endswith(('.xls', '.xlsx')):
                excel_files.append(input_path)
            else:
                print(f"Skipping {input_path}: Not an Excel file or folder")
        else:
            print(f"Skipping {input_path}: Path not found")

    return excel_files

def get_files_from_folder(folder_path):
    """Retrieve all Excel files from the specified folder."""
    if not os.path.exists(folder_path):
        print(f"Folder not found: {folder_path}")
        return []

    # Include all files in the folder and filter for Excel files
    all_files = glob.glob(os.path.join(folder_path, "*"))
    excel_files = [file for file in all_files if file.endswith(('.xls', '.xlsx'))]

    if not excel_files:
        print(f"No Excel files found in folder: {folder_path}")
    return excel_files

def main():
    parser = argparse.ArgumentParser(description='Analyze rugby head motion data from Excel files.')
    parser.add_argument('inputs', nargs='*', help='Excel files or folders containing Excel files to process')
    parser.add_argument('-o', '--output', help='Output CSV filename for summary results')
    args = parser.parse_args()

    excel_files = []

    # Check if the input is a folder or file
    for input_path in args.inputs:
        if os.path.isdir(input_path):
            excel_files.extend(get_files_from_folder(input_path))
        elif os.path.isfile(input_path) and input_path.lower().endswith('.xlsx'):
            excel_files.append(input_path)
        else:
            print(f"Skipping invalid input: {input_path}")

    # If no files were provided as arguments, prompt user for files interactively
    if not excel_files:
        excel_files = get_files_interactively()

    if not excel_files:
        print("No valid Excel files provided.")
        return

    print(f"Found {len(excel_files)} Excel files to process.")

    all_peak_lin_accs = []
    all_peak_rot_accs = []
    results_data = [] # For storing per-file results for CSV export

    for file_path in excel_files:
        print(f"\nProcessing {os.path.basename(file_path)}...")
        peak_lin_acc, peak_rot_acc = process_file(file_path)

        if peak_lin_acc is not None and peak_rot_acc is not None:
            print(f"  Peak Linear Acceleration: {peak_lin_acc:.2f} m/s^2")
            print(f"  Peak Rotational Acceleration: {peak_rot_acc:.2f} rad/s^2")
            all_peak_lin_accs.append(peak_lin_acc)
            all_peak_rot_accs.append(peak_rot_acc)
            results_data.append({
                'file': os.path.basename(file_path),
                'peak_linear_acc_mps2': peak_lin_acc,
                'peak_rotational_acc_rads2': peak_rot_acc
            })
        else:
            print(f"  Could not calculate peak accelerations for {os.path.basename(file_path)}.")

    if not results_data:
        print("\nNo data processed successfully from any file.")
        return

    # Create a DataFrame from the results for easier statistics and export
    results_df = pd.DataFrame(results_data)

    print("\n\n--- Aggregate Statistics ---")
    if not results_df.empty:
        print("\nPeak Linear Accelerations (m/s^2):")
        print(f"  Mean:   {results_df['peak_linear_acc_mps2'].mean():.2f}")
        print(f"  Std Dev:{results_df['peak_linear_acc_mps2'].std():.2f}")
        print(f"  Min:    {results_df['peak_linear_acc_mps2'].min():.2f}")
        print(f"  Max:    {results_df['peak_linear_acc_mps2'].max():.2f}")
        print(f"  Median: {results_df['peak_linear_acc_mps2'].median():.2f}")
        print(f"  Count:  {results_df['peak_linear_acc_mps2'].count()}")

        print("\nPeak Rotational Accelerations (rad/s^2):")
        print(f"  Mean:   {results_df['peak_rotational_acc_rads2'].mean():.2f}")
        print(f"  Std Dev:{results_df['peak_rotational_acc_rads2'].std():.2f}")
        print(f"  Min:    {results_df['peak_rotational_acc_rads2'].min():.2f}")
        print(f"  Max:    {results_df['peak_rotational_acc_rads2'].max():.2f}")
        print(f"  Median: {results_df['peak_rotational_acc_rads2'].median():.2f}")
        print(f"  Count:  {results_df['peak_rotational_acc_rads2'].count()}")

        # Output detailed results to a CSV file
        if args.output:
            output_filename = args.output
        else:
            output_filename = "rugby_head_motion_peak_accelerations_summary.csv"

        try:
            results_df.to_csv(output_filename, index=False, float_format='%.4f')
            print(f"\nDetailed results saved to: {os.path.abspath(output_filename)}")
        except Exception as e:
            print(f"\nError saving detailed results to CSV: {e}")
    else:
        print("\nNo valid peak acceleration data to aggregate.")

if __name__ == "__main__":
    main()
