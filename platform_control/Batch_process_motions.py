import os
import argparse
import pandas as pd
from process_kinematics import calculate_position_orientation
from platform_controller import PlatformController


def batch_process(
    input_dir: str,
    leg_length: float,
    rail_max_travel: float,
    apply_optimization: bool,
    num_cores: int = None
) -> None:
    """
    Batch process all Excel files in a directory: compute kinematics and platform motions.

    Args:
        input_dir: Path to folder containing .xlsx files
        leg_length: Length of each leg on the platform (m)
        rail_max_travel: Maximum slider travel on rails (m)
        apply_optimization: Whether to optimise offsets before computing trajectories
        num_cores: Number of CPU cores to use (default: all available)
    """
    # Find all Excel files
    excel_files = [f for f in os.listdir(input_dir) if f.lower().endswith('.xlsx')]
    if not excel_files:
        print(f"No Excel files found in '{input_dir}'")
        return

    controller = PlatformController(leg_length, rail_max_travel)
    expected_cols = {
        't(ms)': 'time',
        'X(m)': 'x',
        'Y(m)': 'y',
        'Z(m)': 'z',
        'Roll(deg)': 'roll',
        'Pitch(deg)': 'pitch',
        'Yaw(deg)': 'yaw'
    }

    for fname in excel_files:
        file_path = os.path.join(input_dir, fname)
        print(f"\n=== Processing '{fname}' ===")
        try:
            # --- Kinematics ---
            raw_df = pd.read_excel(file_path)
            kin_df = calculate_position_orientation(raw_df)

            # Write Position_Orientation sheet
            with pd.ExcelWriter(file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
                kin_df.to_excel(writer, sheet_name='Position_Orientation', index=False)

            # --- Prepare trajectory DataFrame ---
            traj_df = kin_df.rename(columns=expected_cols)
            # Convert ms to seconds
            traj_df['time'] = traj_df['time'] / 1000.0

            # --- Platform motion ---
            print(f"Computing platform motions (optimisation={'on' if apply_optimization else 'off'})...")
            results_df = controller.process_trajectory(
                traj_df,
                apply_optimization=apply_optimization,
                num_cores=num_cores
            )

            # --- Export results ---
            output_name = fname.replace('.xlsx', '_trajectory_results.xlsx')
            output_path = os.path.join(input_dir, output_name)
            with pd.ExcelWriter(output_path, engine='openpyxl') as writer:
                # Positions
                pos = pd.DataFrame({
                    'Time (s)': results_df['time'],
                    'Slider 1 (m)': results_df['slider1'],
                    'Slider 2 (m)': results_df['slider2'],
                    'Slider 3 (m)': results_df['slider3'],
                    'Motor Angle (deg)': results_df['motor_angle'],
                    'Platform X (m)': results_df['x'],
                    'Platform Y (m)': results_df['y'],
                    'Platform Z (m)': results_df['z'],
                    'Roll (deg)': results_df['roll'],
                    'Pitch (deg)': results_df['pitch'],
                    'Yaw (deg)': results_df['yaw']
                })
                pos.to_excel(writer, sheet_name='Positions', index=False)

                # Velocities
                vel = pd.DataFrame({
                    'Time (s)': results_df['time'],
                    'Slider 1 (m/s)': results_df['velocity1'],
                    'Slider 2 (m/s)': results_df['velocity2'],
                    'Slider 3 (m/s)': results_df['velocity3']
                })
                vel.to_excel(writer, sheet_name='Velocities', index=False)

                # Accelerations
                acc = pd.DataFrame({
                    'Time (s)': results_df['time'],
                    'Slider 1 (m/s²)': results_df['acceleration1'],
                    'Slider 2 (m/s²)': results_df['acceleration2'],
                    'Slider 3 (m/s²)': results_df['acceleration3']
                })
                acc.to_excel(writer, sheet_name='Accelerations', index=False)

                # Statistics
                stats = {
                    'Peak Velocity (m/s)': results_df[['velocity1','velocity2','velocity3']].abs().max().max(),
                    'Peak Acceleration (m/s²)': results_df[['acceleration1','acceleration2','acceleration3']].abs().max().max(),
                    'Total Points': len(results_df)
                }
                stats_df = pd.DataFrame.from_dict(stats, orient='index', columns=['Value'])
                stats_df.to_excel(writer, sheet_name='Statistics')

            print(f"Results written to '{output_name}'")

        except Exception as e:
            print(f"Error processing '{fname}': {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Batch process Excel files: compute kinematics and platform motions"
    )
    parser.add_argument(
        '--input_dir', '-i',
        default='.',
        help='Directory containing Excel files (default: current directory)'
    )
    parser.add_argument(
        '--leg_length', '-l',
        type=float,
        default=0.3,
        help='Leg length in metres (default: 0.3)'
    )
    parser.add_argument(
        '--rail_max_travel', '-r',
        type=float,
        default=0.5,
        help='Rail max travel in metres (default: 0.5)'
    )
    parser.add_argument(
        '--optimize', '-o',
        action='store_true',
        help='Apply optimisation of platform offsets'
    )
    parser.add_argument(
        '--num_cores', '-n',
        type=int,
        default=None,
        help='Number of CPU cores to use (default: all available)'
    )
    args = parser.parse_args()

    batch_process(
        input_dir=args.input_dir,
        leg_length=args.leg_length,
        rail_max_travel=args.rail_max_travel,
        apply_optimization=args.optimize,
        num_cores=args.num_cores
    )


if __name__ == '__main__':
    main()
