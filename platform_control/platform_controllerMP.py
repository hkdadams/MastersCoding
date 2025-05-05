import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict
import math
from tqdm import tqdm
import os
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from functools import partial
import threading
from multiprocessing import Value


# Thread-safe progress bar
class ThreadSafeCounter:
    def __init__(self):
        self.value = 0
        self.lock = threading.Lock()

    def increment(self):
        with self.lock:
            self.value += 1
            return self.value

# Initialize the attempt_counter as a multiprocessing.Value
attempt_counter = Value('i', 0)  # 'i' indicates an integer value

def chunk_data(data: pd.DataFrame, num_chunks: int) -> List[pd.DataFrame]:
    """Split data into optimal chunk sizes for parallel processing"""
    chunk_size = max(1, len(data) // num_chunks)
    return [data.iloc[i:i + chunk_size] for i in range(0, len(data), chunk_size)]

def optimize_with_initial_guess(x0: np.ndarray, bounds: List[Tuple[float, float]], trajectory_df: pd.DataFrame, controller: 'PlatformController', dt: float) -> Dict:
    """Standalone function to run optimization with a single initial guess"""
    try:
        print(f"\nStarting optimization with initial guess:")
        print(f"x0: {x0}")
        print(f"dt: {dt}")
        print(f"trajectory_df shape: {trajectory_df.shape}")

        # Validate dt
        if dt <= 0:
            raise ValueError("dt must be positive")

        result = minimize(
            fun=controller.objective_function,
            x0=x0,
            args=(trajectory_df, dt),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-6}
        )
        
        # Check if this solution is feasible by looking at controller's best_feasible_solution
        is_feasible = hasattr(controller, 'best_feasible_solution') and controller.best_feasible_solution is not None
        
        return {
            'success': result.success,
            'x': result.x,
            'fun': result.fun,
            'message': result.message,
            'is_feasible': is_feasible,
            'feasible_score': controller.best_feasible_solution['score'] if is_feasible else None
        }
    except Exception as e:
        return {
            'success': False,
            'error': str(e),
            'x': x0,
            'fun': float('inf'),
            'is_feasible': False,
            'feasible_score': None
        }

class PlatformController:
    def __init__(self, leg_length: float, rail_max_travel: float, log_file_path: str, log_attempts: bool = True):
        """
        Initialize the platform controller with geometric parameters
        
        Args:
            leg_length: Length of each leg in meters
            rail_max_travel: Maximum travel distance of each slider in meters
            log_file_path: Path to the debug log file
            log_attempts: Whether to log individual optimization attempts (default: True)
        """
        self.leg_length = leg_length
        self.rail_max_travel = rail_max_travel
        self.L_squared = leg_length * leg_length
        self.log_attempts = log_attempts
        
        # Add offset parameters
        self.position_offset = np.zeros(3)  # [x, y, z] offsets
        self.rotation_offset = np.zeros(3)  # [roll, pitch, yaw] offsets in degrees
        
        # Platform geometry (from original implementation)
        self.PLATFORM_SIDE = 0.2
        self.PLATFORM_RADIUS = self.PLATFORM_SIDE / math.sqrt(3)
        
        # Define rail vectors (120° apart)
        self.rail_vectors = [
            np.array([1.0, 0.0, 0.0]),  # 0°
            np.array([np.cos(2*np.pi/3), np.sin(2*np.pi/3), 0.0]),  # 120°
            np.array([np.cos(4*np.pi/3), np.sin(4*np.pi/3), 0.0])   # 240°
        ]
        
        # Define perpendicular vectors for legs 2 and 3
        # These are perpendicular to both the rail direction and the vertical
        self.perp_vectors = [
            None,  # Leg 1 doesn't have this DOF
            np.array([-np.sin(2*np.pi/3), np.cos(2*np.pi/3), 0.0]),  # 120° + 90°
            np.array([-np.sin(4*np.pi/3), np.cos(4*np.pi/3), 0.0])   # 240° + 90°
        ]
        
        # Platform attachment points in local coordinates
        self.platform_points_local = [
            np.array([self.PLATFORM_RADIUS, 0.0, 0.0]),
            np.array([self.PLATFORM_RADIUS*np.cos(2*np.pi/3), self.PLATFORM_RADIUS*np.sin(2*np.pi/3), 0.0]),
            np.array([self.PLATFORM_RADIUS*np.cos(4*np.pi/3), self.PLATFORM_RADIUS*np.sin(4*np.pi/3), 0.0])
        ]

        self.log_file_path = log_file_path

    def transform_platform_points(self, position: np.ndarray, rotation: np.ndarray) -> List[np.ndarray]:
        """
        Transform platform attachment points from local to world coordinates
        
        Args:
            position: [x, y, z] position of platform center
            rotation: 3x3 rotation matrix
            
        Returns:
            List of transformed platform points in world coordinates
        """
        return [rotation @ p + position for p in self.platform_points_local]

    def calculate_joint_angles(self, platform_points: List[np.ndarray], slider_positions: np.ndarray, rotation_matrix: np.ndarray) -> dict:
        """
        Calculate all joint angles for the mechanism
        
        Args:
            platform_points: List of platform attachment points in world coordinates
            slider_positions: Array of slider positions along their rails
            rotation_matrix: 3x3 rotation matrix for platform orientation
            
        Returns:
            Dictionary containing all joint angles in degrees
        """
        angles = {
            'slider_joints': [],  # Linear DOF
            'leg_slider_joints': [],  # Rotational DOF at slider connection
            'leg_platform_joints': []  # Spherical joint angles at platform
        }
        
        for i in range(3):
            # Get vectors for this leg
            slider_pos = self.rail_vectors[i] * slider_positions[i]
            leg_vector = platform_points[i] - slider_pos
            leg_length = np.linalg.norm(leg_vector)
            leg_unit = leg_vector / leg_length
            
            # 1. Slider joint position (1 DOF)
            angles['slider_joints'].append(slider_positions[i])
            
            # 2. Leg-to-slider joint angles
            if i == 0:
                # Leg 1: 2 DOF
                # a) Vertical plane rotation (XZ plane)
                vertical_angle = np.arctan2(leg_vector[2], leg_vector[0])
                # b) Rotation around leg axis
                leg_axis_rotation = np.arctan2(leg_vector[1], np.sqrt(leg_vector[0]**2 + leg_vector[2]**2))
                angles['leg_slider_joints'].append({
                    'vertical_angle': math.degrees(vertical_angle),
                    'leg_axis_rotation': math.degrees(leg_axis_rotation)
                })
            else:
                # Legs 2 and 3: 3 DOF
                # a) Rotation in plane perpendicular to rail
                rail_perp_angle = np.arctan2(
                    np.dot(leg_vector, np.array([0, 0, 1])),
                    np.dot(leg_vector, self.rail_vectors[i])
                )
                # b) Rotation around leg axis
                leg_axis_rotation = np.arctan2(
                    np.dot(leg_vector, self.perp_vectors[i]),
                    np.sqrt(np.dot(leg_vector, self.rail_vectors[i])**2 + leg_vector[2]**2)
                )
                # c) Rotation around perpendicular axis
                perp_angle = np.arctan2(
                    np.dot(leg_vector, np.array([0, 0, 1])),
                    np.dot(leg_vector, self.perp_vectors[i])
                )
                angles['leg_slider_joints'].append({
                    'rail_perp_angle': math.degrees(rail_perp_angle),
                    'leg_axis_rotation': math.degrees(leg_axis_rotation),
                    'perp_angle': math.degrees(perp_angle)
                })
            
            # 3. Leg-to-platform joint angles (3 DOF each)
            # Convert to spherical coordinates relative to platform local frame
            platform_local = np.linalg.inv(rotation_matrix) @ leg_vector
            r = np.linalg.norm(platform_local)
            theta = np.arccos(platform_local[2] / r)
            phi = np.arctan2(platform_local[1], platform_local[0])
            angles['leg_platform_joints'].append({
                'theta': math.degrees(theta),
                'phi': math.degrees(phi),
                'psi': math.degrees(np.arctan2(leg_vector[1], leg_vector[0]))  # Rotation around connection point
            })
        
        return angles

    def calculate_slider_positions(self, platform_points: List[np.ndarray], time: float = None, platform_pos: np.ndarray = None, platform_rot: np.ndarray = None, debug: bool = True) -> Tuple[np.ndarray, float, dict]:
        """
        Calculate slider positions, motor angle, and all joint angles
        
        Args:
            platform_points: List of platform attachment points in world coordinates
            time: Current time in seconds (optional)
            platform_pos: Platform position [x, y, z] (optional)
            platform_rot: Platform rotation in degrees [roll, pitch, yaw] (optional)
            debug: Whether to print debug information (default: True)
            
        Returns:
            Tuple of (slider_positions, motor_angle, joint_angles)
        """
        slider_positions = np.zeros(3)
        
        # Create rotation matrix from Euler angles if provided
        if platform_rot is not None:
            roll, pitch, yaw = np.radians(platform_rot)
            Rx = np.array([[1, 0, 0],
                          [0, np.cos(roll), -np.sin(roll)],
                          [0, np.sin(roll), np.cos(roll)]])
            Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                          [0, 1, 0],
                          [-np.sin(pitch), 0, np.cos(pitch)]])
            Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])
            rotation_matrix = Rz @ Ry @ Rx
        else:
            rotation_matrix = np.eye(3)
        
        if debug:
            print("\n=== Debug: Slider Position Calculations ===")
            if time is not None:
                print(f"Time: {time:.3f}s")
            if platform_pos is not None:
                print(f"Platform position: ({platform_pos[0]:.3f}, {platform_pos[1]:.3f}, {platform_pos[2]:.3f})m")
            if platform_rot is not None:
                print(f"Platform orientation: (roll={platform_rot[0]:.1f}°, pitch={platform_rot[1]:.1f}°, yaw={platform_rot[2]:.1f}°)")
            print(f"Platform size: {self.PLATFORM_SIDE:.3f}m sides, {self.PLATFORM_RADIUS:.3f}m radius")
            print(f"Leg length: {self.leg_length:.3f}m")
            print(f"Rail max travel: {self.rail_max_travel:.3f}m")
        
        # Calculate slider 1 position and motor angle (along x-axis)
        P0 = platform_points[0]
        if debug:
            print(f"\nLeg 1 (X-axis) at t={time:.3f}s:" if time is not None else "\nLeg 1 (X-axis):")
            print(f"  Platform attachment point: ({P0[0]:.3f}, {P0[1]:.3f}, {P0[2]:.3f})m")
        
        dx_sq_component = self.L_squared - P0[2]**2
        if dx_sq_component < 0:
            if debug:
                print(f"  ERROR: Position unreachable - leg length constraint violated")
                print(f"  Required leg length would be: {np.sqrt(P0[0]**2 + P0[1]**2 + P0[2]**2):.3f}m")
            raise ValueError("Position unreachable: leg length constraint violated")
        
        dx_component = -np.sqrt(dx_sq_component)
        slider_positions[0] = P0[0] - dx_component
        motor_angle = np.arctan2(P0[2], dx_component)
        
        if debug:
            print(f"  Horizontal distance component: {abs(dx_component):.3f}m")
            print(f"  Vertical distance component: {P0[2]:.3f}m")
            print(f"  Calculated slider position: {slider_positions[0]:.3f}m")
            print(f"  Motor angle: {math.degrees(motor_angle):.1f}°")
        
        # Calculate slider positions for trucks 2 and 3
        for i in range(1, 3):
            Pi = platform_points[i]
            ui = self.rail_vectors[i]
            angle = 120 if i == 1 else 240
            
            if debug:
                print(f"\nLeg {i+1} ({angle}°) at t={time:.3f}s:" if time is not None else f"\nLeg {i+1} ({angle}°):")
                print(f"  Platform attachment point: ({Pi[0]:.3f}, {Pi[1]:.3f}, {Pi[2]:.3f})m")
                print(f"  Rail direction: ({ui[0]:.3f}, {ui[1]:.3f}, {ui[2]:.3f})")
            
            Pi_dot_ui = np.dot(Pi, ui)
            Pi_dot_Pi = np.dot(Pi, Pi)
            
            if debug:
                print(f"  P·u: {Pi_dot_ui:.3f}")
                print(f"  |P|²: {Pi_dot_Pi:.3f}")
                print(f"  L²: {self.L_squared:.3f}")
            
            discriminant = Pi_dot_ui**2 - (Pi_dot_Pi - self.L_squared)
            if debug:
                print(f"  Discriminant: {discriminant:.3f}")
            
            if discriminant < 0:
                if debug:
                    print(f"  ERROR: Position unreachable for leg {i+1} (discriminant < 0)")
                raise ValueError(f"Position unreachable for leg {i+1}")
                
            # Try both positive and negative roots
            s_pos = Pi_dot_ui + np.sqrt(discriminant)
            s_neg = Pi_dot_ui - np.sqrt(discriminant)
            
            if debug:
                print(f"  Possible slider positions:")
                print(f"    Positive root: {s_pos:.3f}m")
                print(f"    Negative root: {s_neg:.3f}m")
            
            # Choose the root that gives a valid slider position
            if 0 <= s_pos <= self.rail_max_travel:
                slider_positions[i] = s_pos
                if debug:
                    print(f"  Selected positive root: {s_pos:.3f}m (within range [0, {self.rail_max_travel:.3f}])")
            elif 0 <= s_neg <= self.rail_max_travel:
                slider_positions[i] = s_neg
                if debug:
                    print(f"  Selected negative root: {s_neg:.3f}m (within range [0, {self.rail_max_travel:.3f}])")
            else:
                if debug:
                    print(f"  ERROR: Neither root is within valid range [0, {self.rail_max_travel:.3f}]")
                    print(f"    s_pos = {s_pos:.3f}m")
                    print(f"    s_neg = {s_neg:.3f}m")
                raise ValueError(f"Slider {i+1} position out of range")
            
            # Calculate actual leg length for verification
            slider_pos = ui * slider_positions[i]
            leg_vector = Pi - slider_pos
            actual_length = np.linalg.norm(leg_vector)
            if debug:
                print(f"  Verification:")
                print(f"    Slider position: ({slider_pos[0]:.3f}, {slider_pos[1]:.3f}, {slider_pos[2]:.3f})m")
                print(f"    Leg vector: ({leg_vector[0]:.3f}, {leg_vector[1]:.3f}, {leg_vector[2]:.3f})m")
                print(f"    Actual leg length: {actual_length:.3f}m (should be {self.leg_length:.3f}m)")
                print(f"    Length error: {(actual_length - self.leg_length)*1000:.2f}mm")
        
        if debug:
            print(f"\nFinal slider positions at t={time:.3f}s:" if time is not None else "\nFinal slider positions:", [f"{pos:.3f}m" for pos in slider_positions])
            print("=== End Debug Output ===\n")
            
            # Calculate all joint angles
            joint_angles = self.calculate_joint_angles(platform_points, slider_positions, rotation_matrix)
            
            print(f"\n=== Joint Angles at t={time:.3f}s ===" if time is not None else "\n=== Joint Angles ===")
            for i in range(3):
                print(f"\nLeg {i+1}:")
                print(f"  Slider position: {joint_angles['slider_joints'][i]:.3f}m")
                
                if i == 0:
                    angles = joint_angles['leg_slider_joints'][i]
                    print("  Leg-to-slider joint:")
                    print(f"    Vertical angle: {angles['vertical_angle']:.1f}°")
                    print(f"    Leg axis rotation: {angles['leg_axis_rotation']:.1f}°")
                else:
                    angles = joint_angles['leg_slider_joints'][i]
                    print("  Leg-to-slider joint:")
                    print(f"    Rail perpendicular angle: {angles['rail_perp_angle']:.1f}°")
                    print(f"    Leg axis rotation: {angles['leg_axis_rotation']:.1f}°")
                    print(f"    Perpendicular axis angle: {angles['perp_angle']:.1f}°")
                
                platform_angles = joint_angles['leg_platform_joints'][i]
                print("  Leg-to-platform joint:")
                print(f"    θ (elevation): {platform_angles['theta']:.1f}°")
                print(f"    φ (azimuth): {platform_angles['phi']:.1f}°")
                print(f"    ψ (rotation): {platform_angles['psi']:.1f}°")
        else:
            # Calculate joint angles without debug output
            joint_angles = self.calculate_joint_angles(platform_points, slider_positions, rotation_matrix)
        
        return slider_positions, math.degrees(motor_angle), joint_angles

    def optimize_platform_orientation(self, target_position: np.ndarray, target_rotation: np.ndarray, time: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Optimize platform orientation to minimize slider movements while maintaining target position
        
        Args:
            target_position: [x, y, z] target position
            target_rotation: 3x3 target rotation matrix
            time: Current time in seconds (optional)
            
        Returns:
            Tuple of (optimized_position, optimized_rotation)
        """
        def objective(params):
            # params: [x, y, z, rx, ry, rz] (euler angles in radians)
            position = params[:3]
            angles_rad = params[3:]
            
            # Create rotation matrix from Euler angles
            Rx = np.array([[1, 0, 0],
                          [0, np.cos(angles_rad[0]), -np.sin(angles_rad[0])],
                          [0, np.sin(angles_rad[0]), np.cos(angles_rad[0])]])
            Ry = np.array([[np.cos(angles_rad[1]), 0, np.sin(angles_rad[1])],
                          [0, 1, 0],
                          [-np.sin(angles_rad[1]), 0, np.cos(angles_rad[1])]])
            Rz = np.array([[np.cos(angles_rad[2]), -np.sin(angles_rad[2]), 0],
                          [np.sin(angles_rad[2]), np.cos(angles_rad[2]), 0],
                          [0, 0, 1]])
            rotation = Rz @ Ry @ Rx
            angles_deg = np.degrees(angles_rad)
            
            try:
                platform_points = self.transform_platform_points(position, rotation)
                slider_positions, motor_angle, _ = self.calculate_slider_positions(
                    platform_points, 
                    time=time,
                    platform_pos=position,
                    platform_rot=angles_deg,
                    debug=False
                )
                
                # Minimize deviation from target position and orientation
                pos_error = np.linalg.norm(position - target_position)
                rot_error = np.linalg.norm(rotation - target_rotation, ord='fro')
                
                return pos_error + rot_error
            except ValueError:
                return 1e6  # Return large value for invalid configurations
        
        # Initial guess: use target position and rotation
        initial_euler = Rotation.from_matrix(target_rotation).as_euler('xyz')
        initial_guess = np.concatenate([target_position, initial_euler])
        
        # Optimize
        result = minimize(objective, initial_guess, method='SLSQP')
        
        if not result.success:
            raise ValueError("Failed to optimize platform orientation")
        
        optimized_position = result.x[:3]
        optimized_rotation = Rotation.from_euler('xyz', result.x[3:]).as_matrix()
        
        return optimized_position, optimized_rotation

    def optimize_offsets(self, trajectory_df: pd.DataFrame, log_file_path: str, num_cores: int = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Optimize position and rotation offsets with feasibility tracking
        """
        # Get dt from trajectory data
        dt = trajectory_df['time'].diff().iloc[1]
        
        print("Entering optimize_offsets...")
        print(f"Trajectory DataFrame shape: {trajectory_df.shape}")
        print(f"Number of CPU cores: {num_cores}")
        print(f"Using time step dt = {dt:.6f} seconds")
        
        if num_cores is None:
            num_cores = mp.cpu_count()

        # Initialize best feasible solution tracker
        self.best_feasible_solution = None
        best_result = None
        best_score = float('inf')
        feasible_solutions_found = 0
        
        # Get minimum and maximum z positions from trajectory
        min_z_traj = trajectory_df['z'].min()
        max_z_traj = trajectory_df['z'].max()
        
        # Calculate z offset bounds to ensure positive z values
        z_offset_min = max(0, -min_z_traj)  # Force minimum z offset to be non-negative
        z_offset_max = self.leg_length - max_z_traj
        z_mid = (z_offset_min + z_offset_max) / 2
        
        bounds = [
            (-self.leg_length, self.leg_length),  # x offset: ±leg_length
            (-0.0, 0.0),  # y offset
            (z_offset_min, z_offset_max),  # z offset with new bounds (always non-negative)
            (-40, 40),    # roll offset (degrees)
            (-40, 40),    # pitch offset (degrees)
            (-40, 40)     # yaw offset (degrees)
        ]
        
        print(f"\nOptimization bounds:")
        print(f"  X offset: ±{self.leg_length:.3f}m")
        print(f"  Y offset: ±0m")
        print(f"  Z offset: [{z_offset_min:.3f}m, {z_offset_max:.3f}m]")
        print(f"  Z mid-point: {z_mid:.3f}m")
        print(f"  Rotation offsets: ±40 degrees")
        print(f"\nTrajectory ranges:")
        print(f"  Original z range: [{min_z_traj:.3f}m, {max_z_traj:.3f}m]")
        print(f"\nUsing {num_cores} CPU cores for optimization")
        
        # Create systematic initial guesses
        initial_guesses = [
            np.array([0.0, 0.0, z_mid, 0.0, 0.0, 0.0]),     # All centered
            np.array([self.leg_length/2, 0.0, z_mid, 20.0, 20.0, 20.0]),   # Positive half-range
            np.array([-self.leg_length/2, 0.0, z_mid, -20.0, -20.0, -20.0]), # Negative half-range
            np.array([0.0, 0.0, z_mid*1.2, 0.0, 0.0, 0.0]),  # Slight higher Z
            np.array([0.0, 0.0, z_mid*0.8, 0.0, 0.0, 0.0])  # Slight lower Z
        ]
        
        print("Submitting optimization tasks...")

        # Run optimizations in parallel
        with ProcessPoolExecutor(max_workers=num_cores) as executor:
            futures = []
            for x0 in initial_guesses:
                future = executor.submit(
                    optimize_with_initial_guess,
                    x0=x0,
                    bounds=bounds,
                    trajectory_df=trajectory_df,
                    controller=self,
                    dt=dt
                )
                futures.append(future)
            
            # Process results as they complete
            for future in tqdm(as_completed(futures), total=len(initial_guesses), 
                             desc="Optimizing offsets", unit="attempt"):
                try:
                    result = future.result()
                    if result['success']:
                        feasible_solutions_found += 1
                        if result['fun'] < best_score:
                            best_score = result['fun']
                            best_result = result
                            # Update best feasible solution
                            self.best_feasible_solution = {
                                'score': best_score,
                                'position_offset': result['x'][:3],
                                'rotation_offset': result['x'][3:],
                                'peak_velocity': result.get('peak_velocity', 0),
                                'peak_acceleration': result.get('peak_acceleration', 0)
                            }
                            print(f"\nFound better solution:")
                            print(f"  Score: {best_score:.3f}")
                            print(f"  Position offset: ({result['x'][0]:.3f}, {result['x'][1]:.3f}, {result['x'][2]:.3f})m")
                            print(f"  Rotation offset: ({result['x'][3]:.1f}, {result['x'][4]:.1f}, {result['x'][5]:.1f})°")
                except Exception as e:
                    print(f"\nOptimization attempt failed: {str(e)}")
                    continue

        print("Optimization tasks completed.")
        print(f"Feasible solutions found: {feasible_solutions_found}")
        if best_result:
            print(f"Best result score: {best_result['fun']}")
        else:
            print("No valid results found.")
        
        print(f"\nOptimization complete:")
        print(f"  Feasible solutions found: {feasible_solutions_found}")
        
        if self.best_feasible_solution:
            print("\nUsing best feasible solution:")
            print(f"  Score: {self.best_feasible_solution['score']:.3f}")
            print(f"  Position offset: ({self.best_feasible_solution['position_offset'][0]:.3f}, "
                  f"{self.best_feasible_solution['position_offset'][1]:.3f}, "
                  f"{self.best_feasible_solution['position_offset'][2]:.3f})m")
            print(f"  Rotation offset: ({self.best_feasible_solution['rotation_offset'][0]:.1f}, "
                  f"{self.best_feasible_solution['rotation_offset'][1]:.1f}, "
                  f"{self.best_feasible_solution['rotation_offset'][2]:.1f})°")

            # Ensure the optimal offsets are assigned to the self object
            self.position_offset = self.best_feasible_solution['position_offset']
            self.rotation_offset = self.best_feasible_solution['rotation_offset']

            return self.best_feasible_solution['position_offset'], self.best_feasible_solution['rotation_offset']
        else:
            print("\nWarning: No feasible solution found")
            if best_result is None:
                # Return zeros if no solution found at all
                return np.zeros(3), np.zeros(3)
            return best_result['x'][:3], best_result['x'][3:]  # Access dictionary properly

    def objective_function(self, params: np.ndarray, trajectory_df: pd.DataFrame, dt: float) -> float:
        """
        Objective function for optimization that strongly prefers feasible solutions.
        Includes scaled penalties for out-of-bounds positions and tracks feasible solutions.
        
        Args:
            params: Array of [x_offset, y_offset, z_offset, roll_offset, pitch_offset, yaw_offset]
            trajectory_df: DataFrame with trajectory data
            dt: Time step between trajectory points
            
        Returns:
            float: Objective value (lower is better)
        """

        pos_offset = params[:3]
        rot_offset = params[3:]
        
        # Store original offsets
        orig_pos_offset = self.position_offset.copy()
        orig_rot_offset = self.rotation_offset.copy()

        try:
            total_error = 0
            unreachable_count = 0
            out_of_bounds_count = 0
            max_out_of_bounds = 0

            # Initialize list to store all slider positions
            all_slider_positions = []
            
            # Process each point in the trajectory
            for _, row in trajectory_df.iterrows():
                try:
                    # Apply offsets
                    position = np.array([
                        row['x'] + pos_offset[0],
                        row['y'] + pos_offset[1],
                        row['z'] + pos_offset[2]
                    ])
                    angles = np.array([
                        row['roll'] + rot_offset[0],
                        row['pitch'] + rot_offset[1],
                        row['yaw'] + rot_offset[2]
                    ])
                    
                    # Create rotation matrix and calculate platform points
                    roll, pitch, yaw = np.radians(angles)
                    Rx = np.array([[1, 0, 0],
                                [0, np.cos(roll), -np.sin(roll)],
                                [0, np.sin(roll), np.cos(roll)]])
                    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                                [0, 1, 0],
                                [-np.sin(pitch), 0, np.cos(pitch)]])
                    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                [np.sin(yaw), np.cos(yaw), 0],
                                [0, 0, 1]])
                    rotation = Rz @ Ry @ Rx
                    
                    platform_points = self.transform_platform_points(position, rotation)
                    
                    # Try to calculate slider positions
                    slider_positions, _, _ = self.calculate_slider_positions(
                        platform_points,
                        platform_pos=position,
                        platform_rot=angles,
                        debug=False
                    )
                    
                    # Store slider positions
                    all_slider_positions.append(slider_positions)
                    
                    # Check bounds and calculate penalties
                    for pos in slider_positions:
                        if pos < 0:
                            out_of_bounds_count += 1
                            max_out_of_bounds = max(max_out_of_bounds, abs(pos))
                        elif pos > self.rail_max_travel:
                            out_of_bounds_count += 1
                            max_out_of_bounds = max(max_out_of_bounds, pos - self.rail_max_travel)
                    
                    # Add movement cost (only if point is reachable)
                    total_error += np.sum(np.abs(slider_positions)) * 0.01
                    
                except Exception:
                    unreachable_count += 1

            # Calculate velocities and accelerations if we have positions
            if len(all_slider_positions) > 1:
                all_positions = np.array(all_slider_positions)
                velocities = np.diff(all_positions, axis=0) / dt
                peak_velocity = np.max(np.abs(velocities))
                
                if len(velocities) > 1:
                    accelerations = np.diff(velocities, axis=0) / dt
                    peak_acceleration = np.max(np.abs(accelerations))
                else:
                    peak_acceleration = 0
            else:
                peak_velocity = 0
                peak_acceleration = 0

            # Add velocity and acceleration penalties to the score
            velocity_penalty = peak_velocity * 10.0 if peak_velocity > 2.0 else 0.0  # Penalize velocities above 2 m/s
            
            # Add harsh penalty for exceeding maximum acceleration of 50 m/s²
            if peak_acceleration > 50.0:
                acceleration_penalty = (peak_acceleration - 50.0) * 100.0  # Much higher penalty factor for exceeding max
            else:
                acceleration_penalty = peak_acceleration * 5.0 if peak_acceleration > 10.0 else 0.0  # Original acceleration penalty
            
            # Calculate final score
            if unreachable_count > 0 or out_of_bounds_count > 0:
                score = 1e6 + (unreachable_count * 1e4) + (out_of_bounds_count * 1e3) + (max_out_of_bounds * 1e2)
            else:
                # Solution is completely feasible, use movement cost and dynamic penalties
                score = total_error + velocity_penalty + acceleration_penalty
                
                # Store this feasible solution if it's the best so far
                if not self.best_feasible_solution or score < self.best_feasible_solution['score']:
                    self.best_feasible_solution = {
                        'score': score,
                        'position_offset': pos_offset.copy(),
                        'rotation_offset': rot_offset.copy(),
                        'peak_velocity': peak_velocity,
                        'peak_acceleration': peak_acceleration
                    }

            if self.log_attempts:
                # Get the attempt number from the counter
                with attempt_counter.get_lock():
                    attempt_number = attempt_counter.value
                    attempt_counter.value += 1

            # Log each optimization attempt to a debug file with additional metrics
                try:
                    with open(self.log_file_path, "a") as debug_file:
                        debug_file.write(f"Attempt: {attempt_number}, Score: {score}, Position Offset: {pos_offset}, Rotation Offset: {rot_offset}, Peak Velocity: {peak_velocity}, Peak Acceleration: {peak_acceleration}\n")
                except Exception as e:
                    print(f"Error writing to debug log: {e}")

            return score
                        
        except Exception as e:
            print(f"Error in objective_function: {e}")
            return float('inf')
        
        finally:
            # Restore original offsets
            self.position_offset = orig_pos_offset
            self.rotation_offset = orig_rot_offset

    def process_trajectory(self, trajectory_df: pd.DataFrame, file_path: str, apply_optimization: bool = True, auto_overwrite: bool = True) -> pd.DataFrame:
        """
        Process trajectory data from DataFrame and compute slider positions and motor angles.
        Avoids splitting into chunks to prevent boundary errors.

        Args:
            trajectory_df: DataFrame with columns ['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
                         angles in degrees
            file_path: Path to the input Excel file
            apply_optimization: Whether to apply stored offsets
            auto_overwrite: If True, automatically overwrite existing files without prompting

        Returns:
            DataFrame with computed slider positions and motor angles
        """
        dt = trajectory_df['time'].diff().iloc[1]

        # Create output directory structure
        output_dir = os.path.join(os.path.dirname(file_path), "platform_outputs")
        os.makedirs(output_dir, exist_ok=True)

        # Create the output file name based on the input file name
        input_file_name = os.path.basename(file_path).replace('.xlsx', '')
        output_file = os.path.join(output_dir, f"{input_file_name}_platform_motion.xlsx")
        
        all_results = []

        print("\nProcessing trajectory without chunking")
        prev_sliders = None

        for _, row in trajectory_df.iterrows():
            try:
                # Apply offsets if optimization is enabled
                if apply_optimization:
                    position = np.array([
                        row['x'] + self.position_offset[0],
                        row['y'] + self.position_offset[1],
                        row['z'] + self.position_offset[2]
                    ])
                    angles = np.array([
                        row['roll'] + self.rotation_offset[0],
                        row['pitch'] + self.rotation_offset[1],
                        row['yaw'] + self.rotation_offset[2]
                    ])
                else:
                    position = np.array([row['x'], row['y'], row['z']])
                    angles = np.array([row['roll'], row['pitch'], row['yaw']])

                # Create rotation matrix from angles
                roll, pitch, yaw = np.radians(angles)
                Rx = np.array([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]])
                Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                             [0, 1, 0],
                             [-np.sin(pitch), 0, np.cos(pitch)]])
                Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                             [np.sin(yaw), np.cos(yaw), 0],
                             [0, 0, 1]])
                rotation = Rz @ Ry @ Rx

                # Optimize platform orientation
                opt_position, opt_rotation = self.optimize_platform_orientation(position, rotation, time=row['time'])
                opt_angles = Rotation.from_matrix(opt_rotation).as_euler('xyz', degrees=True)

                # Calculate slider positions and motor angle
                platform_points = self.transform_platform_points(opt_position, opt_rotation)
                slider_positions, motor_angle, joint_angles = self.calculate_slider_positions(
                    platform_points, 
                    time=row['time'],
                    platform_pos=opt_position,
                    platform_rot=opt_angles,
                    debug=False
                )

                # Calculate velocities and accelerations
                if prev_sliders is not None:
                    velocities = (slider_positions - prev_sliders) / dt
                    if len(all_results) > 0:
                        prev_velocities = all_results[-1]['velocities']
                        accelerations = (velocities - prev_velocities) / dt
                    else:
                        accelerations = np.zeros(3)
                else:
                    velocities = np.zeros(3)
                    accelerations = np.zeros(3)

                all_results.append({
                    'time': row['time'],
                    'slider_positions': slider_positions,
                    'motor_angle': motor_angle,
                    'velocities': velocities,
                    'accelerations': accelerations,
                    'joint_angles': joint_angles,
                    'platform_position': opt_position,
                    'platform_rotation': opt_angles,
                    'x': opt_position[0],
                    'y': opt_position[1],
                    'z': opt_position[2],
                    'roll': opt_angles[0],
                    'pitch': opt_angles[1],
                    'yaw': opt_angles[2],
                    'slider1': slider_positions[0],
                    'slider2': slider_positions[1],
                    'slider3': slider_positions[2],
                    'velocity1': velocities[0],
                    'velocity2': velocities[1],
                    'velocity3': velocities[2],
                    'acceleration1': accelerations[0],
                    'acceleration2': accelerations[1],
                    'acceleration3': accelerations[2]
                })

                prev_sliders = slider_positions

            except ValueError as e:
                print(f"Error processing row: {e}")
                continue

        # Sort results by time
        all_results.sort(key=lambda x: x['time'])

        # Create DataFrame with results
        results_df = pd.DataFrame(all_results)

        # Update the export logic to use the new output file name
        print(f"\nExporting results to {os.path.abspath(output_file)}...")
        with ThreadPoolExecutor() as executor:
            executor.submit(self.export_results_to_excel, results_df, output_file, auto_overwrite)

        print(f"\nResults successfully exported to {os.path.abspath(output_file)}")
        print("\nKey Statistics:")
        print(f"Peak Velocity: {results_df['velocity1'].max():.3f} m/s")
        print(f"Peak Acceleration: {results_df['acceleration1'].max():.3f} m/s²")

        return results_df

    def export_results_to_excel(self, results_df: pd.DataFrame, output_file: str, auto_overwrite: bool = True):
        """Export results to Excel file"""
        try:
            # Create an output directory
            output_dir = os.path.join(os.path.dirname(output_file), "platform_outputs")
            os.makedirs(output_dir, exist_ok=True)
            
            # Update output file path to be in the output directory
            base_name = os.path.basename(output_file)
            if not base_name.endswith('_platform_motion.xlsx'):
                base_name = base_name.replace('.xlsx', '_platform_motion.xlsx')
            output_file = os.path.join(output_dir, base_name)
            
            # Check if file exists and warn user
            if os.path.exists(output_file) and not auto_overwrite:
                user_input = input(f"\nWarning: {os.path.basename(output_file)} already exists. Overwrite? (y/N): ")
                if not user_input.lower().startswith('y'):
                    print("Export cancelled. Please provide a different filename.")
                    return
            
            # First write the final optimization summary to the debug log
            debug_log_path = os.path.join(output_dir, f"debug_log_{os.path.basename(output_file).replace('_platform_motion.xlsx', '')}.txt")
            with open(debug_log_path, "a") as debug_file:
                debug_file.write("\n\nFINAL OPTIMIZATION RESULTS\n")
                debug_file.write("=========================\n\n")
                
                # Best Score
                debug_file.write("Optimization Score:\n")
                if self.best_feasible_solution:
                    debug_file.write(f"Best Feasible Score: {self.best_feasible_solution['score']:.6f}\n\n")
                else:
                    debug_file.write("No feasible solution found\n\n")
                
                # Platform Motion Ranges
                debug_file.write("Platform Motion Ranges:\n")
                debug_file.write(f"X Range: {results_df['x'].min():.3f} to {results_df['x'].max():.3f} m\n")
                debug_file.write(f"Y Range: {results_df['y'].min():.3f} to {results_df['y'].max():.3f} m\n")
                debug_file.write(f"Z Range: {results_df['z'].min():.3f} to {results_df['z'].max():.3f} m\n")
                debug_file.write(f"Roll Range: {results_df['roll'].min():.1f}deg to {results_df['roll'].max():.1f}deg\n")
                debug_file.write(f"Pitch Range: {results_df['pitch'].min():.1f}deg to {results_df['pitch'].max():.1f}deg\n")
                debug_file.write(f"Yaw Range: {results_df['yaw'].min():.1f}deg to {results_df['yaw'].max():.1f}deg\n\n")
                
                # Slider Positions
                debug_file.write("Slider Positions:\n")
                debug_file.write(f"Slider 1: {results_df['slider1'].min():.3f} to {results_df['slider1'].max():.3f} m\n")
                debug_file.write(f"Slider 2: {results_df['slider2'].min():.3f} to {results_df['slider2'].max():.3f} m\n")
                debug_file.write(f"Slider 3: {results_df['slider3'].min():.3f} to {results_df['slider3'].max():.3f} m\n\n")
                
                # Dynamic Properties
                debug_file.write("Peak Velocities:\n")
                debug_file.write(f"Slider 1: {results_df['velocity1'].abs().max():.3f} m/s\n")
                debug_file.write(f"Slider 2: {results_df['velocity2'].abs().max():.3f} m/s\n")
                debug_file.write(f"Slider 3: {results_df['velocity3'].abs().max():.3f} m/s\n\n")
                
                debug_file.write("Peak Accelerations:\n")
                debug_file.write(f"Slider 1: {results_df['acceleration1'].abs().max():.3f} m/s^2\n")
                debug_file.write(f"Slider 2: {results_df['acceleration2'].abs().max():.3f} m/s^2\n")
                debug_file.write(f"Slider 3: {results_df['acceleration3'].abs().max():.3f} m/s^2\n\n")
                
                # Final Optimization Results
                debug_file.write("Optimization Results:\n")
                debug_file.write(f"Position Offset: ({self.position_offset[0]:.3f}, {self.position_offset[1]:.3f}, {self.position_offset[2]:.3f}) m\n")
                debug_file.write(f"Rotation Offset: ({self.rotation_offset[0]:.1f}, {self.rotation_offset[1]:.1f}, {self.rotation_offset[2]:.1f})deg\n\n")
                debug_file.write("=========================\n")

            # Now proceed with Excel export
            with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
                # Main results sheet with added angles
                main_results = pd.DataFrame({
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
                main_results.to_excel(writer, sheet_name='Positions', index=False)

                # Velocities sheet
                velocities = pd.DataFrame({
                    'Time (s)': results_df['time'],
                    'Slider 1 (m/s)': results_df['velocity1'],
                    'Slider 2 (m/s)': results_df['velocity2'],
                    'Slider 3 (m/s)': results_df['velocity3']
                })
                velocities.to_excel(writer, sheet_name='Velocities', index=False)

                # Accelerations sheet
                accelerations = pd.DataFrame({
                    'Time (s)': results_df['time'],
                    'Slider 1 (m/s²)': results_df['acceleration1'],
                    'Slider 2 (m/s²)': results_df['acceleration2'],
                    'Slider 3 (m/s²)': results_df['acceleration3']
                })
                accelerations.to_excel(writer, sheet_name='Accelerations', index=False)

                # Statistics sheet with expanded metrics
                stats = pd.DataFrame({
                    'Metric': [
                        'Peak Velocity (m/s)',
                        'Peak Acceleration (m/s²)',
                        'Max Slider 1 Position (m)',
                        'Max Slider 2 Position (m)',
                        'Max Slider 3 Position (m)',
                        'Min Slider 1 Position (m)',
                        'Min Slider 2 Position (m)',
                        'Min Slider 3 Position (m)',
                        'Slider 1 Range (m)',
                        'Slider 2 Range (m)',
                        'Slider 3 Range (m)',
                        'Position Offset X (m)',
                        'Position Offset Y (m)',
                        'Position Offset Z (m)',
                        'Rotation Offset Roll (deg)',
                        'Rotation Offset Pitch (deg)',
                        'Rotation Offset Yaw (deg)',
                        'Platform X Range (m)',
                        'Platform Y Range (m)',
                        'Platform Z Range (m)',
                        'Roll Range (deg)',
                        'Pitch Range (deg)',
                        'Yaw Range (deg)',
                        'Total Points',
                        'Unreachable Points',
                        'Success Rate (%)'
                    ],
                    'Value': [
                        np.max([np.abs(results_df['velocity1'].max()), np.abs(results_df['velocity2'].max()), np.abs(results_df['velocity3'].max())]),
                        np.max([np.abs(results_df['acceleration1'].max()), np.abs(results_df['acceleration2'].max()), np.abs(results_df['acceleration3'].max())]),
                        results_df['slider1'].max(),
                        results_df['slider2'].max(),
                        results_df['slider3'].max(),
                        results_df['slider1'].min(),
                        results_df['slider2'].min(),
                        results_df['slider3'].min(),
                        results_df['slider1'].max() - results_df['slider1'].min(),
                        results_df['slider2'].max() - results_df['slider2'].min(),
                        results_df['slider3'].max() - results_df['slider3'].min(),
                        self.position_offset[0],
                        self.position_offset[1],
                        self.position_offset[2],
                        self.rotation_offset[0],
                        self.rotation_offset[1],
                        self.rotation_offset[2],
                        results_df['x'].max() - results_df['x'].min(),
                        results_df['y'].max() - results_df['y'].min(),
                        results_df['z'].max() - results_df['z'].min(),
                        results_df['roll'].max() - results_df['roll'].min(),
                        results_df['pitch'].max() - results_df['pitch'].min(),
                        results_df['yaw'].max() - results_df['yaw'].min(),
                        len(results_df),
                        0,  # Placeholder for unreachable points
                        100.0  # Placeholder for success rate
                    ]
                })
                stats.to_excel(writer, sheet_name='Statistics', index=False)

            print(f"Results successfully exported to {output_file}")
        except PermissionError:
            print(f"\nError: Cannot save results - the file '{output_file}' is currently open.")
            print("Please close the Excel file and run the program again.")
            return
        except FileNotFoundError:
            print(f"\nError: File '{output_file}' not found. Please check the file path.")
            return
        except Exception as e:
            print(f"Error exporting results to Excel: {e}")

    def plot_results(self, results_df: pd.DataFrame):
        """Plot the trajectory results"""
        fig, axs = plt.subplots(3, 1, figsize=(10, 12))
        
        # Plot slider positions
        times = results_df['time']
        axs[0].plot(times, [p[0] for p in results_df['slider_positions']], 'b-', label='Slider 1')
        axs[0].plot(times, [p[1] for p in results_df['slider_positions']], 'g-', label='Slider 2')
        axs[0].plot(times, [p[2] for p in results_df['slider_positions']], 'r-', label='Slider 3')
        axs[0].set_ylabel('Position (m)')
        axs[0].set_title('Slider Positions')
        axs[0].legend()
        axs[0].grid(True)
        
        # Plot velocities with points to show data gaps
        axs[1].plot(times, [v[0] for v in results_df['velocities']], 'b.-', label='Slider 1')
        axs[1].plot(times, [v[1] for v in results_df['velocities']], 'g.-', label='Slider 2')
        axs[1].plot(times, [v[2] for v in results_df['velocities']], 'r.-', label='Slider 3')
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].set_title('Slider Velocities')
        axs[1].legend()
        axs[1].grid(True)
        
        # Plot accelerations with points to show data gaps
        axs[2].plot(times, [a[0] for a in results_df['accelerations']], 'b.-', label='Slider 1')
        axs[2].plot(times, [a[1] for a in results_df['accelerations']], 'g.-', label='Slider 2')
        axs[2].plot(times, [a[2] for a in results_df['accelerations']], 'r.-', label='Slider 3')
        axs[2].set_ylabel('Acceleration (m/s²)')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_title('Slider Accelerations')
        axs[2].legend()
        axs[2].grid(True)
        
        plt.tight_layout()
        plt.show()

def main():
    # Get the Excel file path from user input
    while True:
        file_path = input("Enter the path to your Excel file: ").strip()
        try:
            # Try to read the Excel file and specifically the Position_Orientation sheet
            trajectory_df = pd.read_excel(file_path, sheet_name='Position_Orientation')
            
            # Define expected column headers
            expected_columns = {
                't(ms)': 'time',
                'X(m)': 'x',
                'Y(m)': 'y',
                'Z(m)': 'z',
                'Roll(deg)': 'roll',
                'Pitch(deg)': 'pitch',
                'Yaw(deg)': 'yaw'
            }
            
            # Check for missing columns
            missing_columns = [col for col in expected_columns.keys() if col not in trajectory_df.columns]
            
            if missing_columns:
                print(f"Error: Missing required columns: {', '.join(missing_columns)}")
                print("The Position_Orientation sheet must contain: t(ms), X(m), Y(m), Z(m), Roll(deg), Pitch(deg), Yaw(deg)")
                continue
            
            # Rename columns to internal names and convert time from ms to seconds
            trajectory_df = trajectory_df.rename(columns=expected_columns)
            trajectory_df['time'] = trajectory_df['time'] / 1000.0  # Convert ms to seconds
                
            break
        except FileNotFoundError:
            print(f"Error: File '{file_path}' not found. Please enter a valid file path.")
        except ValueError as e:
            if "Position_Orientation" in str(e):
                print("Error: Sheet 'Position_Orientation' not found in the Excel file.")
            else:
                print(f"Error reading Excel file: {str(e)}")
        except Exception as e:
            print(f"Error: {str(e)}")
    
    # Create output directory in the same folder as the input file
    output_dir = os.path.join(os.path.dirname(file_path), "platform_outputs")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory created/verified: {output_dir}")

    # Define the debug log file path in the output directory
    input_file_name = os.path.basename(file_path).replace('.xlsx', '')
    log_file_path = os.path.join(output_dir, f"debug_log_{input_file_name}.txt")
    print(f"Debug log file path set: {log_file_path}")

    # Create the debug log file to ensure it exists
    try:
        if not os.path.exists(log_file_path):
            with open(log_file_path, "w") as debug_file:
                debug_file.write("Debug log initialized.\n")
            print(f"Debug log file created: {log_file_path}")
        else:
            print(f"Debug log file already exists: {log_file_path}")
    except Exception as e:
        print(f"Failed to create debug log file: {e}")

    # Get geometric parameters from user
    while True:
        try:
            leg_length = float(input("Enter leg length in meters (default 0.3): ") or "0.3")
            if leg_length <= 0:
                raise ValueError("Leg length must be positive")
            break
        except ValueError:
            print("Please enter a valid positive number")
    
    while True:
        try:
            rail_max_travel = float(input("Enter maximum rail travel in meters (default 0.5): ") or "0.5")
            if rail_max_travel <= 0:
                raise ValueError("Rail travel must be positive")
            break
        except ValueError:
            print("Please enter a valid positive number")
    
    # Ask if user wants to log optimization attempts
    log_attempts = input("Log optimization attempts? (y/N): ").lower().strip() == 'y'
    
    # Create controller with specified parameters
    controller = PlatformController(leg_length, rail_max_travel, log_file_path, log_attempts=log_attempts)
    
    # First process trajectory without optimization
    print("\nProcessing trajectory without optimization...")
    results_df_no_opt = controller.process_trajectory(trajectory_df, file_path=file_path, apply_optimization=False)
    
    # Get initial position without optimization
    initial_pos_no_opt = results_df_no_opt['platform_position'].iloc[0]
    initial_rot_no_opt = results_df_no_opt['platform_rotation'].iloc[0]
    
    # Calculate peak values without optimization
    peak_velocities_no_opt = np.max([np.abs(v) for v in results_df_no_opt['velocities']])
    peak_accelerations_no_opt = np.max([np.abs(a) for a in results_df_no_opt['accelerations']])
    
    print("\nNon-optimized results:")
    print(f"Initial position (x, y, z): ({initial_pos_no_opt[0]:.3f}, {initial_pos_no_opt[1]:.3f}, {initial_pos_no_opt[2]:.3f}) m")
    print(f"Initial orientation (roll, pitch, yaw): ({initial_rot_no_opt[0]:.1f}, {initial_rot_no_opt[1]:.1f}, {initial_rot_no_opt[2]:.1f}) degrees")
    print(f"Peak velocity: {peak_velocities_no_opt:.3f} m/s")
    print(f"Peak acceleration: {peak_accelerations_no_opt:.3f} m/s²")
    
    print("\nOptimizing trajectory offsets...")
    # Calculate dt from trajectory data
    dt = trajectory_df['time'].diff().iloc[1]
    print(f"Using time step dt = {dt:.6f} seconds")
    
    # Optimize offsets with proper parameters
    pos_offset, rot_offset = controller.optimize_offsets(
        trajectory_df=trajectory_df,
        log_file_path=log_file_path,
        num_cores=None  # Will use default value
    )
    
    # Set optimized offsets
    controller.position_offset = pos_offset
    controller.rotation_offset = rot_offset
    
    print("\nOptimized offsets:")
    print(f"Position offset (x, y, z): ({pos_offset[0]:.3f}, {pos_offset[1]:.3f}, {pos_offset[2]:.3f}) m")
    print(f"Rotation offset (roll, pitch, yaw): ({rot_offset[0]:.1f}, {rot_offset[1]:.1f}, {rot_offset[2]:.1f}) degrees")
    
    print("\nProcessing trajectory with optimized offsets...")
    # Process trajectory with optimized offsets
    results_df = controller.process_trajectory(trajectory_df, file_path=file_path)
    
    # Get initial position with optimization
    initial_pos_opt = results_df['platform_position'].iloc[0]
    initial_rot_opt = results_df['platform_rotation'].iloc[0]
    
    # Calculate peak values with optimization
    peak_velocities = np.max([np.abs(v) for v in results_df['velocities']])
    peak_accelerations = np.max([np.abs(a) for a in results_df['accelerations']])
    
    print("\nOptimized results:")
    print(f"Initial position (x, y, z): ({initial_pos_opt[0]:.3f}, {initial_pos_opt[1]:.3f}, {initial_pos_opt[2]:.3f}) m")
    print(f"Initial orientation (roll, pitch, yaw): ({initial_rot_opt[0]:.1f}, {initial_rot_opt[1]:.1f}, {initial_rot_opt[2]:.1f}) degrees")
    print(f"Peak velocity: {peak_velocities:.3f} m/s")
    print(f"Peak acceleration: {peak_accelerations:.3f} m/s²")
    
    print("\nImprovement:")
    vel_improvement = (peak_velocities_no_opt - peak_velocities) / peak_velocities_no_opt * 100
    acc_improvement = (peak_accelerations_no_opt - peak_accelerations) / peak_accelerations_no_opt * 100
    print(f"Velocity reduction: {vel_improvement:.1f}%")
    print(f"Acceleration reduction: {acc_improvement:.1f}%")
    
    # Plot results
    controller.plot_results(results_df)

if __name__ == "__main__":
    main()