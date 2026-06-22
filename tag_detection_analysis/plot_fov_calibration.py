#!/usr/bin/env python3

import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def main():
    # ---------------- Setup Directories ----------------
    base_dir = os.path.expanduser('~/humble_docker_ws/px4_ros2_ws/src/px4_vision_hardware_cpp/tag_detection_analysis')
    # Fallback to local desktop if not in docker
    if not os.path.exists(base_dir):
        base_dir = os.path.expanduser('~/Desktop/px4_vision_hardware_cpp/tag_detection_analysis')
        
    results_dir = os.path.join(base_dir, 'results')
    os.makedirs(results_dir, exist_ok=True)

    # Find the most recent CSV file
    csv_files = glob.glob(os.path.join(base_dir, '*.csv'))
    if not csv_files:
        print("[ERROR] No CSV files found in the directory.")
        return
    
    latest_csv = max(csv_files, key=os.path.getctime)
    print(f"\n[INFO] Loading data from: {os.path.basename(latest_csv)}")

    # Load Data
    df = pd.read_csv(latest_csv)
    
    # Normalize timestamp to start at 0 seconds
    df['time_s'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Clean data (drop any rows where MoCap didn't initialize yet)
    df = df[df['mocap_alt_m'] > -1.0].copy()

    print(f"[INFO] Processed {len(df)} frames of flight data.")

    # ---------------------------------------------------------
    # PLOT A: Visibility vs Altitude
    # ---------------------------------------------------------
    print("\n[ANALYSIS] Generating Plot A: Visibility vs Altitude...")
    plt.figure(figsize=(10, 6))
    
    # Jitter the Y-axis slightly so the dots don't completely overlap
    plt.scatter(df['mocap_alt_m'].to_numpy(), (df['bundle_visible'] * 1.05).to_numpy(), 
                color='blue', alpha=0.5, label='Outer Bundle Visible', s=10)
    plt.scatter(df['mocap_alt_m'].to_numpy(), (df['tag_19_visible'] * 0.95).to_numpy(), 
                color='red', alpha=0.5, label='Tag 19 Visible', s=10)

    plt.axvline(x=0.6, color='green', linestyle='--', label='Hardcoded 0.6m Threshold')
    
    plt.title('Plot A: Tag Visibility vs. MoCap Altitude', fontsize=14, fontweight='bold')
    plt.xlabel('MoCap Altitude (m)', fontsize=12)
    plt.ylabel('Visibility (Boolean)', fontsize=12)
    plt.yticks([0, 1], ['Not Visible', 'Visible'])
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plot_a_path = os.path.join(results_dir, 'Plot_A_Visibility_vs_Altitude.png')
    plt.savefig(plot_a_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ---------------------------------------------------------
    # PLOT B: PX4 vs MoCap Altitude
    # ---------------------------------------------------------
    print("[ANALYSIS] Generating Plot B: Estimator Drift Analysis...")
    plt.figure(figsize=(10, 6))
    
    # ADDED .to_numpy() here to fix the ValueError
    plt.plot(df['time_s'].to_numpy(), df['mocap_alt_m'].to_numpy(), label='MoCap Ground Truth', color='black', linewidth=2)
    plt.plot(df['time_s'].to_numpy(), df['px4_alt_m'].to_numpy(), label='PX4 EKF Estimate', color='orange', linestyle='--', linewidth=2)
    
    # Calculate Noise/Error
    df['alt_error'] = np.abs(df['mocap_alt_m'] - df['px4_alt_m'])
    mean_error = df['alt_error'].mean()
    max_error = df['alt_error'].max()
    print(f"    -> Mean Altitude Error (PX4 vs MoCap): {mean_error:.3f} m")
    print(f"    -> Max Altitude Error (PX4 vs MoCap): {max_error:.3f} m")

    plt.title('Plot B: PX4 EKF vs OptiTrack Ground Truth', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Altitude (m)', fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plot_b_path = os.path.join(results_dir, 'Plot_B_PX4_vs_MoCap.png')
    plt.savefig(plot_b_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ---------------------------------------------------------
    # PLOT C: Detection Rate Heatmap (5cm Bins)
    # ---------------------------------------------------------
    print("[ANALYSIS] Generating Plot C: 5cm Binned Detection Rates...")
    
    # Create 5cm bins
    max_alt = df['mocap_alt_m'].max()
    bins = np.arange(0, max_alt + 0.05, 0.05)
    df['alt_bin'] = pd.cut(df['mocap_alt_m'], bins)
    
    # Calculate percentages
    binned_bundle = df.groupby('alt_bin', observed=False)['bundle_visible'].mean() * 100
    binned_tag19 = df.groupby('alt_bin', observed=False)['tag_19_visible'].mean() * 100
    
    # Extract center of each bin for plotting
    bin_centers = [b.mid for b in binned_bundle.index]

    plt.figure(figsize=(12, 6))
    width = 0.02
    plt.bar([x - width/2 for x in bin_centers], binned_bundle.to_numpy(), width=width, label='Outer Bundle', color='blue', alpha=0.7)
    plt.bar([x + width/2 for x in bin_centers], binned_tag19.to_numpy(), width=width, label='Tag 19', color='red', alpha=0.7)

    plt.title('Plot C: Detection Reliability per 5cm Altitude Band', fontsize=14, fontweight='bold')
    plt.xlabel('MoCap Altitude Bins (m)', fontsize=12)
    plt.ylabel('Detection Rate (%)', fontsize=12)
    plt.axvline(x=0.6, color='green', linestyle='--', label='Current 0.6m Threshold')
    plt.legend()
    plt.grid(axis='y', alpha=0.3)
    
    plot_c_path = os.path.join(results_dir, 'Plot_C_Detection_Heatmap.png')
    plt.savefig(plot_c_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ---------------------------------------------------------
    # PLOT D: Switching Event Audit (Hysteresis Simulation)
    # ---------------------------------------------------------
    print("[ANALYSIS] Generating Plot D: Hysteresis Logic Audit...")
    
    # Simulate your C++ LandingDirector logic
    tracking_small_tag = False
    director_states = []
    
    for idx, row in df.iterrows():
        alt = row['mocap_alt_m']
        tag_19_vis = row['tag_19_visible']
        
        # Exact logic from your C++ node
        if alt <= 0.60 and tag_19_vis == 1:
            tracking_small_tag = True
        elif alt > 0.80:
            tracking_small_tag = False
            
        director_states.append(tracking_small_tag)
        
    df['director_state'] = director_states

    plt.figure(figsize=(12, 6))
    plt.plot(df['time_s'].to_numpy(), df['mocap_alt_m'].to_numpy(), color='black', linewidth=2, label='Drone Trajectory')
    
    # Shade background based on director state. Added .to_numpy() here too!
    plt.fill_between(df['time_s'].to_numpy(), 0, df['mocap_alt_m'].max() + 0.2, 
                     where=(df['director_state'] == False).to_numpy(), 
                     color='blue', alpha=0.1, label='Active: Outer Bundle')
    
    plt.fill_between(df['time_s'].to_numpy(), 0, df['mocap_alt_m'].max() + 0.2, 
                     where=(df['director_state'] == True).to_numpy(), 
                     color='red', alpha=0.1, label='Active: Tag 19')

    plt.axhline(y=0.60, color='red', linestyle=':', label='Descent Threshold (0.6m)')
    plt.axhline(y=0.80, color='blue', linestyle=':', label='Ascent Threshold (0.8m)')

    plt.title('Plot D: System State Hysteresis Overlay', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('MoCap Altitude (m)', fontsize=12)
    
    # Clean up legend
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='upper right')
    
    plot_d_path = os.path.join(results_dir, 'Plot_D_Switching_Audit.png')
    plt.savefig(plot_d_path, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"\n[SUCCESS] All 4 plots generated and saved to: {results_dir}")

if __name__ == '__main__':
    main()