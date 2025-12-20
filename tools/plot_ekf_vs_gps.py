import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np

# Path to log file
csv_path = os.path.expanduser("~/webots_ws_new/logs/ekf_gps_log.csv")

# Load CSV
df = pd.read_csv(csv_path)

# 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(df["ekf_x"].values, df["ekf_y"].values, df["ekf_z"].values, label="EKF Trajectory", linewidth=2)
ax.plot(df["gps_x"].values, df["gps_y"].values, df["gps_z"].values, label="GPS Trajectory", linestyle='dashed')
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3D Trajectory: EKF vs GPS")
ax.legend()
plt.tight_layout()
plt.show(block=False)


gps_error = np.sqrt((df["ekf_x"].values - df["gps_x"].values)**2 +
                    (df["ekf_y"].values - df["gps_y"].values)**2 +
                    (df["ekf_z"].values - df["gps_z"].values)**2)

plt.figure()
plt.plot(df["time"].to_numpy(), gps_error, color='red')
plt.title("GPS Position Error Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Position Error (meters)")
plt.grid(True)
plt.tight_layout()
plt.show(block=False)

input("Enter to close")
