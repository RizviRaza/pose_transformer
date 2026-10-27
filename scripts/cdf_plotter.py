import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# -----------------------------
# Global font size
# -----------------------------
plt.rcParams.update({
    "font.size": 25,
    "axes.titlesize": 30,
    "axes.labelsize": 25,
    "xtick.labelsize": 25,
    "ytick.labelsize": 25,
    "legend.fontsize": 20
})

# -----------------------------
# Indoor Data
# -----------------------------
x_errors = np.array([0.0489, 0.0644, 0.0683, 0.0433, 0.0336,
                     0.0055, 0.0672, 0.0477, 0.0611, 0.0804])
y_errors = np.array([0.0317, 0.0628, 0.0756, 0.0356, 0.0470,
                     0.0341, 0.0382, 0.0330, 0.0309, 0.0461])
z_errors = np.array([0.0461, 0.0438, 0.0496, 0.0498, 0.0742,
                     0.0584, 0.0792, 0.0489, 0.0457, 0.0283])
euclidean_errors = np.array([0.0743, 0.1001, 0.1133, 0.0750, 0.0940,
                             0.0679, 0.1106, 0.0758, 0.0823, 0.0969])
yaw_errors_deg = np.array([0.7918, 1.5453, 1.5829, 2.1361, 2.1123,
                           1.0335, 1.2019, 1.2101, 1.5954, 1.0776])


# -----------------------------
# Outdoor Data
# -----------------------------

# x_errors = np.array([0.0792, 0.0725, 0.0873, 0.0648, 0.0827,
#                      0.0701, 0.0856, 0.0734, 0.0779, 0.0895])

# y_errors = np.array([0.0661, 0.0618, 0.0554, 0.0712, 0.0683,
#                      0.0739, 0.0602, 0.0657, 0.0586, 0.0721])

# z_errors = np.array([0.0734, 0.0687, 0.0816, 0.0772, 0.0721,
#                      0.0698, 0.0804, 0.0711, 0.0768, 0.0823])

# euclidean_errors = np.array([0.1203, 0.1127, 0.1229, 0.1162, 0.1197,
#                              0.1160, 0.1209, 0.1148, 0.1164, 0.1270])

# yaw_errors_deg = np.array([2.2412, 1.9841, 2.4527, 1.8734, 2.2945,
#                            2.0106, 2.3761, 2.1889, 2.3215, 2.5013])


# RMSEs
rmse_euclid = np.sqrt(np.mean(euclidean_errors**2))
rmse_yaw    = np.sqrt(np.mean(yaw_errors_deg**2))

# Medians
median_x, median_y, median_z = np.median(x_errors), np.median(y_errors), np.median(z_errors)
median_yaw = np.median(yaw_errors_deg)
median_euclid = np.median(euclidean_errors)

# -----------------------------
# Figure 1: Positional (X,Y,Z) errors only (with Median + RMSE lines)
# -----------------------------
fig1, ax1 = plt.subplots(figsize=(6,4))

ax1.boxplot([x_errors, y_errors, z_errors],
            labels=["X (m)", "Y (m)", "Z (m)"],
            patch_artist=True,
            medianprops=dict(color="red", linewidth=3.5))
ax1.set_ylabel("Error (m)")
ax1.set_title("Positional Errors (X, Y, Z) (Indoor)")
ax1.grid(axis='y', linestyle='--', alpha=0.6)

# Compute RMSEs
rmse_x = np.sqrt(np.mean(x_errors**2))
rmse_y = np.sqrt(np.mean(y_errors**2))
rmse_z = np.sqrt(np.mean(z_errors**2))

# Draw RMSE lines at positions 1, 2, 3
ax1.hlines(rmse_x, 0.75, 1.25, colors="blue", linestyles="--", linewidth=2)
ax1.hlines(rmse_y, 1.75, 2.25, colors="blue", linestyles="--", linewidth=2)
ax1.hlines(rmse_z, 2.75, 3.25, colors="blue", linestyles="--", linewidth=2)

# Legend handles with medians
median_handles_xyz = [
    mlines.Line2D([], [], color="red", linewidth=3.5, label=f"Median X = {median_x:.3f} m"),
    mlines.Line2D([], [], color="red", linewidth=3.5, label=f"Median Y = {median_y:.3f} m"),
    mlines.Line2D([], [], color="red", linewidth=3.5, label=f"Median Z = {median_z:.3f} m")
]

# Legend handles with RMSEs
rmse_handles_xyz = [
    mlines.Line2D([], [], color="blue", linestyle="--", linewidth=2, label=f"RMSE X = {rmse_x:.3f} m"),
    mlines.Line2D([], [], color="blue", linestyle="--", linewidth=2, label=f"RMSE Y = {rmse_y:.3f} m"),
    mlines.Line2D([], [], color="blue", linestyle="--", linewidth=2, label=f"RMSE Z = {rmse_z:.3f} m")
]

# Add legend
ax1.legend(handles=median_handles_xyz + rmse_handles_xyz, loc="upper right")

plt.tight_layout()
plt.savefig("fig1_positional_xyz_median_rmse.png", dpi=300)
plt.show()



# -----------------------------
# Figure 2: Overall RMSE (Euclidean + Yaw)
# -----------------------------
fig2, axes = plt.subplots(1, 2, figsize=(10,4))

# Subplot 1: Overall positional (Euclidean)
axes[0].boxplot(euclidean_errors, patch_artist=True,
                medianprops=dict(color="red", linewidth=3.5))
axes[0].set_xticks([1])
axes[0].set_xticklabels(["Euclidean (m)"])
axes[0].set_ylabel("Error (m)")
axes[0].set_title("Overall Euclidean Error (Indoor)")
rmse_line = axes[0].axhline(rmse_euclid, linestyle='--', color='blue',
                            linewidth=2, label=f"RMSE = {rmse_euclid:.3f} m")
median_handle = mlines.Line2D([], [], color="red", linewidth=3.5,
                              label=f"Median = {median_euclid:.3f} m")
axes[0].legend(handles=[median_handle, rmse_line], loc="upper right")
axes[0].grid(axis='y', linestyle='--', alpha=0.6)

# Subplot 2: Overall rotational (Yaw)
axes[1].boxplot(yaw_errors_deg, patch_artist=True,
                medianprops=dict(color="red", linewidth=3.5))
axes[1].set_xticks([1])
axes[1].set_xticklabels(["Yaw (°)"])
axes[1].set_ylabel("Error (°)")
axes[1].set_title("Overall Orientation Error (Indoor)")
rmse_line2 = axes[1].axhline(rmse_yaw, linestyle='--', color='blue',
                             linewidth=2, label=f"RMSE = {rmse_yaw:.2f}°")
median_handle2 = mlines.Line2D([], [], color="red", linewidth=3.5,
                               label=f"Median = {median_yaw:.2f}°")
axes[1].legend(handles=[median_handle2, rmse_line2], loc="upper right")
axes[1].grid(axis='y', linestyle='--', alpha=0.6)

plt.tight_layout()
plt.savefig("fig2_overall_positional_rotational_rmse.png", dpi=300)
plt.show()
