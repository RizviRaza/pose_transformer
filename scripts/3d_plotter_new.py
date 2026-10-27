import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np


def read_excel(file_path, sheet_name='New (2)'):
    return pd.read_excel(file_path, sheet_name=sheet_name, engine='openpyxl')


def prepare_data(data):
    # Keep original column names, but make values numeric where possible
    for col in data.columns:
        data[col] = pd.to_numeric(data[col], errors='coerce')
    data = data.dropna(how='all').reset_index(drop=True)
    return data


def find_column(columns, candidates):
    for c in candidates:
        if c in columns:
            return c
    raise KeyError(f"Could not find any of these columns: {candidates}")


def plot_positional_error_boxplot(data, title='Positional Errors (X, Y, Z) (Indoor)'):
    columns = list(data.columns)

    x_col = find_column(columns, ['Positional Error X (m)', 'Positional Error X'])
    y_col = find_column(columns, ['Positional Error Y (m)', 'Positional Error Y'])
    z_col = find_column(columns, ['Positional Error Z (m)', 'Positional Error Z'])

    rmse_x_col = find_column(columns, ['RMSE Positional Error X (m)', 'RMSE X (m)', 'RMSE Positional Error X'])
    rmse_y_col = find_column(columns, ['RMSE Positional Error Y (m)', 'RMSE Y (m)', 'RMSE Positional Error Y'])
    rmse_z_col = find_column(columns, ['RMSE Positional Error Z (m)', 'RMSE Z (m)', 'RMSE Positional Error Z'])

    x = data[x_col].dropna().to_numpy()
    y = data[y_col].dropna().to_numpy()
    z = data[z_col].dropna().to_numpy()

    rmse_x_vals = data[rmse_x_col].dropna().to_numpy()
    rmse_y_vals = data[rmse_y_col].dropna().to_numpy()
    rmse_z_vals = data[rmse_z_col].dropna().to_numpy()

    rmse_x = float(rmse_x_vals[0]) if len(rmse_x_vals) > 0 else np.sqrt(np.mean(x**2))
    rmse_y = float(rmse_y_vals[0]) if len(rmse_y_vals) > 0 else np.sqrt(np.mean(y**2))
    rmse_z = float(rmse_z_vals[0]) if len(rmse_z_vals) > 0 else np.sqrt(np.mean(z**2))

    med_x = np.median(x)
    med_y = np.median(y)
    med_z = np.median(z)

    fig, ax = plt.subplots(figsize=(12, 7), dpi=160)

    box = ax.boxplot(
        [x, y, z],
        labels=['X (m)', 'Y (m)', 'Z (m)'],
        patch_artist=True,
        widths=0.30,
        medianprops=dict(color='red', linewidth=2),
        boxprops=dict(facecolor='#1f77b4', alpha=0.95),
        whiskerprops=dict(color='black'),
        capprops=dict(color='black')
    )

    # RMSE short dashed lines
    r1 = ax.hlines(rmse_x, 0.90, 1.10, colors='blue', linestyles='--', linewidth=1.5, label=f'RMSE X = {rmse_x:.4f} m')
    r2 = ax.hlines(rmse_y, 1.90, 2.10, colors='blue', linestyles='--', linewidth=1.5, label=f'RMSE Y = {rmse_y:.4f} m')
    r3 = ax.hlines(rmse_z, 2.90, 3.10, colors='blue', linestyles='--', linewidth=1.5, label=f'RMSE Z = {rmse_z:.4f} m')
    # Median short lines
    h1 = ax.hlines(med_x, 0.82, 1.18, colors='red', linewidth=2, label=f'Median X = {med_x:.4f} m')
    h2 = ax.hlines(med_y, 1.82, 2.18, colors='red', linewidth=2, label=f'Median Y = {med_y:.4f} m')
    h3 = ax.hlines(med_z, 2.82, 3.18, colors='red', linewidth=2, label=f'Median Z = {med_z:.4f} m')

    

    ax.set_title(title, fontsize=22)
    ax.set_ylabel('Error (m)', fontsize=18)
    ax.tick_params(axis='both', labelsize=16)
    ax.grid(True, axis='y', linestyle='--', alpha=0.5)
    ax.legend(loc='upper right', fontsize=14)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    file_path = script_dir / "indoor_experiments.xlsx"

    data = read_excel(file_path, sheet_name='New (2)')
    data = prepare_data(data)

    plot_positional_error_boxplot(data)