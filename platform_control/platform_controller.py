import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from typing import Tuple, List
import math
from tqdm import tqdm
import os

# Import constraint validation functions
try:
    from constraint_validation import validate_4dof_constraints, enforce_4dof_constraints, get_achievable_dof_description
except ImportError:
    print("Warning: constraint_validation module not found. Constraint validation will be limited.")
    def validate_4dof_constraints(*args, **kwargs):
        return True, ""
    def enforce_4dof_constraints(pose):
        pose[1] = 0.0  # Y = 0
        pose[5] = 0.0  # Yaw = 0
        return pose
    def get_achievable_dof_description():
        return "4 DOF: X, Z, Roll, Pitch (Y and Yaw mechanically impossible)"

class PlatformController:
    def __init__(self, leg_length: float, rail_max_travel: float):
        """
        Initialize the platform controller with geometric parameters
        
        Args:
            leg_length: Length of each leg in meters
            rail_max_travel: Maximum travel distance of each slider in meters
        """
        self.leg_length = leg_length
        self.rail_max_travel = rail_max_travel
        self.L_squared = leg_length * leg_length
        
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
            # Validate 4 DOF constraints before calculation
            if platform_pos is not None and platform_rot is not None:
                # Check Y constraint (should be 0)
                if abs(platform_pos[1]) > 1e-6:
                    print(f"WARNING: Y position {platform_pos[1]:.6f}m violates 4 DOF constraint (should be 0)")
                
                # Check Yaw constraint (should be 0)
                if abs(platform_rot[2]) > 1e-6:
                    print(f"WARNING: Yaw rotation {platform_rot[2]:.6f}° violates 4 DOF constraint (should be 0)")

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

    def objective_function(self, params: np.ndarray, trajectory_df: pd.DataFrame, attempt_number: int = None, callback=None) -> float:
        """Objective function for optimization with progress reporting"""
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
            
            # Calculate score with extreme preference for feasible solutions
            if unreachable_count > 0 or out_of_bounds_count > 0:
                score = 1e6 + (unreachable_count * 1e4) + (out_of_bounds_count * 1e3) + (max_out_of_bounds * 1e2)
            else:
                # Solution is completely feasible, use only movement cost
                score = total_error
                
                # Store this feasible solution if it's the best so far
                if not hasattr(self, 'best_feasible_solution') or score < self.best_feasible_solution['score']:
                    self.best_feasible_solution = {
                        'score': score,
                        'position_offset': pos_offset.copy(),
                        'rotation_offset': rot_offset.copy()
                    }
            
            # Report progress if callback provided
            if callback and attempt_number is not None:
                callback(attempt_number, pos_offset, rot_offset, score,
                        self.best_feasible_solution['position_offset'] if hasattr(self, 'best_feasible_solution') else None,
                        self.best_feasible_solution['rotation_offset'] if hasattr(self, 'best_feasible_solution') else None,
                        self.best_feasible_solution['score'] if hasattr(self, 'best_feasible_solution') else None)
            
            return score
            
        except Exception:
            return float('inf')
        
        finally:
            # Restore original offsets
            self.position_offset = orig_pos_offset
            self.rotation_offset = orig_rot_offset

    def optimize_offsets(self, trajectory_df: pd.DataFrame, optimization_callback=None) -> Tuple[np.ndarray, np.ndarray]:
        """Optimize position and rotation offsets with progress reporting"""
        # Initialize best feasible solution tracker
        self.best_feasible_solution = None
        
        # Get minimum and maximum z positions from trajectory
        min_z_traj = trajectory_df['z'].min()
        max_z_traj = trajectory_df['z'].max()
          # Calculate z offset bounds
        z_offset_min = max(-min_z_traj, -self.leg_length/2)
        z_offset_max = min(self.leg_length - max_z_traj, self.leg_length/2)
        
        # Calculate mid-points
        z_mid = (z_offset_min + z_offset_max) / 2
        
        # Bounds for optimization - enforcing 4 DOF constraints
        bounds = [
            (-self.leg_length, self.leg_length),  # x offset: ±leg_length
            (-0.0, 0.0),  # y offset: LOCKED to 0 (mechanically impossible)
            (z_offset_min, z_offset_max),  # z offset with new bounds
            (-40, 40),    # roll offset (degrees)
            (-40, 40),    # pitch offset (degrees)
            (-0.0, 0.0)   # yaw offset: LOCKED to 0 (mechanically impossible)
        ]
        
        print(f"\nOptimization bounds:")
        print(f"  X offset: ±{self.leg_length:.3f}m")
        print(f"  Y offset: 0m (mechanically locked)")
        print(f"  Z offset: [{z_offset_min:.3f}m, {z_offset_max:.3f}m]")
        print(f"  Z mid-point: {z_mid:.3f}m")
        print(f"  Roll/Pitch offsets: ±40 degrees")
        print(f"  Yaw offset: 0° (mechanically locked)")
        print(f"\nTrajectory ranges:")
        print(f"  Original z range: [{min_z_traj:.3f}m, {max_z_traj:.3f}m]")
        
        # Create systematic initial guesses - respecting 4 DOF constraints
        initial_guesses = [
            np.array([0.0, 0.0, z_mid, 0.0, 0.0, 0.0]),     # All centered
            np.array([self.leg_length/2, 0.0, z_mid, 20.0, 20.0, 0.0]),   # Positive half-range (Y,Yaw locked)
            np.array([-self.leg_length/2, 0.0, z_mid, -20.0, -20.0, 0.0]), # Negative half-range (Y,Yaw locked)
            np.array([0.0, 0.0, z_mid*1.1, 0.0, 0.0, 0.0]),  # Higher Z
            np.array([0.0, 0.0, z_mid*0.9, 0.0, 0.0, 0.0])  # Lower Z
        ]
        
        best_result = None
        best_score = float('inf')
        
        # Add progress bar for optimization attempts
        for attempt, x0 in enumerate(initial_guesses, 1):
            result = minimize(
                self.objective_function,
                x0=x0,
                args=(trajectory_df, attempt, optimization_callback),
                method='SLSQP',
                bounds=bounds,
                options={'maxiter': 100}
            )
            
            if result.success and result.fun < best_score:
                best_score = result.fun
                best_result = result
                print(f"\nFound better solution with score: {best_score:.3f}")
                print(f"  Position offset: ({result.x[0]:.3f}, {result.x[1]:.3f}, {result.x[2]:.3f})m")
                print(f"  Rotation offset: ({result.x[3]:.1f}, {result.x[4]:.1f}, {result.x[5]:.1f})°")
        
        # After optimization, check if we found a feasible solution
        if hasattr(self, 'best_feasible_solution') and self.best_feasible_solution is not None:
            print("\nBest feasible solution found:")
            print(f"  Score: {self.best_feasible_solution['score']:.3f}")
            print(f"  Position offset: ({self.best_feasible_solution['position_offset'][0]:.3f}, "
                  f"{self.best_feasible_solution['position_offset'][1]:.3f}, "
                  f"{self.best_feasible_solution['position_offset'][2]:.3f})m")
            print(f"  Rotation offset: ({self.best_feasible_solution['rotation_offset'][0]:.1f}, "
                  f"{self.best_feasible_solution['rotation_offset'][1]:.1f}, "
                  f"{self.best_feasible_solution['rotation_offset'][2]:.1f})°")
            return self.best_feasible_solution['position_offset'], self.best_feasible_solution['rotation_offset']
        else:
            print("\nNo completely feasible solution found")
            return best_result.x[:3], best_result.x[3:]

    def process_trajectory(self, trajectory_df: pd.DataFrame, apply_optimization: bool = True) -> pd.DataFrame:
        """
        Process trajectory data from DataFrame and compute slider positions and motor angles
        
        Args:
            trajectory_df: DataFrame with columns ['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
                         angles in degrees
            apply_optimization: Whether to apply stored offsets
            
        Returns:
            DataFrame with computed slider positions and motor angles
        """
        results = []
        prev_sliders = None
        dt = trajectory_df['time'].diff().iloc[1]  # Time step
        unreachable_count = 0
        unreachable_reasons = {}
        
        # Process each row sequentially
        for _, row in tqdm(trajectory_df.iterrows(), total=len(trajectory_df),
                          desc="Processing trajectory", unit="points"):
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
                    if len(results) > 0:
                        prev_velocities = results[-1]['velocities']
                        accelerations = (velocities - prev_velocities) / dt
                    else:
                        accelerations = np.zeros(3)
                else:
                    velocities = np.zeros(3)
                    accelerations = np.zeros(3)
                
                results.append({
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
                unreachable_count += 1
                error_msg = str(e)
                unreachable_reasons[error_msg] = unreachable_reasons.get(error_msg, 0) + 1
                continue
        
        # Create DataFrame with results
        results_df = pd.DataFrame(results)
        
        # Export results to Excel with multiple sheets
        output_file = 'trajectory_results.xlsx'
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
                'Yaw (deg)': results_df['yaw'],
                'Leg 1 Vertical Angle (deg)': [angles['leg_slider_joints'][0]['vertical_angle'] for angles in results_df['joint_angles']],
                'Leg 1 Axis Rotation (deg)': [angles['leg_slider_joints'][0]['leg_axis_rotation'] for angles in results_df['joint_angles']],
                'Leg 2 Rail Angle (deg)': [angles['leg_slider_joints'][1]['rail_perp_angle'] for angles in results_df['joint_angles']],
                'Leg 2 Axis Rotation (deg)': [angles['leg_slider_joints'][1]['leg_axis_rotation'] for angles in results_df['joint_angles']],
                'Leg 2 Perp Angle (deg)': [angles['leg_slider_joints'][1]['perp_angle'] for angles in results_df['joint_angles']],
                'Leg 3 Rail Angle (deg)': [angles['leg_slider_joints'][2]['rail_perp_angle'] for angles in results_df['joint_angles']],
                'Leg 3 Axis Rotation (deg)': [angles['leg_slider_joints'][2]['leg_axis_rotation'] for angles in results_df['joint_angles']],
                'Leg 3 Perp Angle (deg)': [angles['leg_slider_joints'][2]['perp_angle'] for angles in results_df['joint_angles']]
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
            
            # New Angles sheet with detailed joint angles
            angles_data = {
                'Time (s)': results_df['time'],
                # Leg 1 angles
                'Leg 1 Vertical Angle (deg)': [angles['leg_slider_joints'][0]['vertical_angle'] for angles in results_df['joint_angles']],
                'Leg 1 Axis Rotation (deg)': [angles['leg_slider_joints'][0]['leg_axis_rotation'] for angles in results_df['joint_angles']],
                'Leg 1 Platform θ (deg)': [angles['leg_platform_joints'][0]['theta'] for angles in results_df['joint_angles']],
                'Leg 1 Platform φ (deg)': [angles['leg_platform_joints'][0]['phi'] for angles in results_df['joint_angles']],
                'Leg 1 Platform ψ (deg)': [angles['leg_platform_joints'][0]['psi'] for angles in results_df['joint_angles']],
                # Leg 2 angles
                'Leg 2 Rail Angle (deg)': [angles['leg_slider_joints'][1]['rail_perp_angle'] for angles in results_df['joint_angles']],
                'Leg 2 Axis Rotation (deg)': [angles['leg_slider_joints'][1]['leg_axis_rotation'] for angles in results_df['joint_angles']],
                'Leg 2 Perp Angle (deg)': [angles['leg_slider_joints'][1]['perp_angle'] for angles in results_df['joint_angles']],
                'Leg 2 Platform θ (deg)': [angles['leg_platform_joints'][1]['theta'] for angles in results_df['joint_angles']],
                'Leg 2 Platform φ (deg)': [angles['leg_platform_joints'][1]['phi'] for angles in results_df['joint_angles']],
                'Leg 2 Platform ψ (deg)': [angles['leg_platform_joints'][1]['psi'] for angles in results_df['joint_angles']],
                # Leg 3 angles
                'Leg 3 Rail Angle (deg)': [angles['leg_slider_joints'][2]['rail_perp_angle'] for angles in results_df['joint_angles']],
                'Leg 3 Axis Rotation (deg)': [angles['leg_slider_joints'][2]['leg_axis_rotation'] for angles in results_df['joint_angles']],
                'Leg 3 Perp Angle (deg)': [angles['leg_slider_joints'][2]['perp_angle'] for angles in results_df['joint_angles']],
                'Leg 3 Platform θ (deg)': [angles['leg_platform_joints'][2]['theta'] for angles in results_df['joint_angles']],
                'Leg 3 Platform φ (deg)': [angles['leg_platform_joints'][2]['phi'] for angles in results_df['joint_angles']],
                'Leg 3 Platform ψ (deg)': [angles['leg_platform_joints'][2]['psi'] for angles in results_df['joint_angles']]
            }
            angles_df = pd.DataFrame(angles_data)
            angles_df.to_excel(writer, sheet_name='Angles', index=False)
            
            # Statistics sheet
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
                    'Position Offset X (m)',
                    'Position Offset Y (m)',
                    'Position Offset Z (m)',
                    'Rotation Offset Roll (deg)',
                    'Rotation Offset Pitch (deg)',
                    'Rotation Offset Yaw (deg)',
                    'Max Leg 1 Vertical Angle (deg)',
                    'Max Leg 2 Rail Angle (deg)',
                    'Max Leg 3 Rail Angle (deg)',
                    'Min Leg 1 Vertical Angle (deg)',
                    'Min Leg 2 Rail Angle (deg)',
                    'Min Leg 3 Rail Angle (deg)'
                ],
                'Value': [
                    np.max([np.abs(v) for v in results_df['velocities']]),
                    np.max([np.abs(a) for a in results_df['accelerations']]),
                    np.max(results_df['slider1']),
                    np.max(results_df['slider2']),
                    np.max(results_df['slider3']),
                    np.min(results_df['slider1']),
                    np.min(results_df['slider2']),
                    np.min(results_df['slider3']),
                    self.position_offset[0],
                    self.position_offset[1],
                    self.position_offset[2],
                    self.rotation_offset[0],
                    self.rotation_offset[1],
                    self.rotation_offset[2],
                    max([angles['leg_slider_joints'][0]['vertical_angle'] for angles in results_df['joint_angles']]),
                    max([angles['leg_slider_joints'][1]['rail_perp_angle'] for angles in results_df['joint_angles']]),
                    max([angles['leg_slider_joints'][2]['rail_perp_angle'] for angles in results_df['joint_angles']]),
                    min([angles['leg_slider_joints'][0]['vertical_angle'] for angles in results_df['joint_angles']]),
                    min([angles['leg_slider_joints'][1]['rail_perp_angle'] for angles in results_df['joint_angles']]),
                    min([angles['leg_slider_joints'][2]['rail_perp_angle'] for angles in results_df['joint_angles']])
                ]
            })
            stats.to_excel(writer, sheet_name='Statistics', index=False)
        
        print(f"\nResults exported to {os.path.abspath(output_file)}")
        
        if unreachable_count > 0:
            print(f"\nTrajectory Analysis:")
            print(f"Total points: {len(trajectory_df)}")
            print(f"Unreachable points: {unreachable_count} ({unreachable_count/len(trajectory_df)*100:.1f}%)")
        
        return results_df

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
    
    # Create controller with specified parameters
    controller = PlatformController(leg_length, rail_max_travel)
    
    # First process trajectory without optimization
    print("\nProcessing trajectory without optimization...")
    results_df_no_opt = controller.process_trajectory(trajectory_df, apply_optimization=False)
    
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
    # Optimize offsets
    pos_offset, rot_offset = controller.optimize_offsets(trajectory_df)
    
    # Set optimized offsets
    controller.position_offset = pos_offset
    controller.rotation_offset = rot_offset
    
    print("\nOptimized offsets:")
    print(f"Position offset (x, y, z): ({pos_offset[0]:.3f}, {pos_offset[1]:.3f}, {pos_offset[2]:.3f}) m")
    print(f"Rotation offset (roll, pitch, yaw): ({rot_offset[0]:.1f}, {rot_offset[1]:.1f}, {rot_offset[2]:.1f}) degrees")
    
    print("\nProcessing trajectory with optimized offsets...")
    # Process trajectory with optimized offsets
    results_df = controller.process_trajectory(trajectory_df)
    
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