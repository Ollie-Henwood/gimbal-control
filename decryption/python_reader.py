import pandas as pd
import matplotlib.pyplot as plt

# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------
file_path = "group9_fly_6D.tsv"
body_name = "group9_multirotor"   # Change if needed

# ------------------------------------------------------------------
# Find the actual table header
# ------------------------------------------------------------------
with open(file_path, "r") as f:
    lines = f.readlines()

header_line_index = None
for i, line in enumerate(lines):
    if line.startswith("Frame\tTime"):
        header_line_index = i
        break

if header_line_index is None:
    raise ValueError("Could not find table header in TSV file.")

# ------------------------------------------------------------------
# Read the TSV table
# ------------------------------------------------------------------
df = pd.read_csv(
    file_path,
    sep="\t",
    skiprows=header_line_index,
    engine="python"
)

df = df.dropna(axis=1, how="all")

# ------------------------------------------------------------------
# Locate body columns
# ------------------------------------------------------------------
columns = list(df.columns)

body_x_col = f"{body_name} X"

if body_x_col not in columns:
    raise ValueError(f"Could not find body '{body_name}' in file.")

start_idx = columns.index(body_x_col)

body_columns = {
    "X": columns[start_idx],
    "Y": columns[start_idx + 1],
    "Z": columns[start_idx + 2],
    "Roll": columns[start_idx + 3],
    "Pitch": columns[start_idx + 4],
    "Yaw": columns[start_idx + 5],
}

# ------------------------------------------------------------------
# Convert columns to numeric
# ------------------------------------------------------------------
for col in ["Time"] + list(body_columns.values()):
    df[col] = pd.to_numeric(df[col], errors="coerce")

# ------------------------------------------------------------------
# Sort by time
# ------------------------------------------------------------------
df = df.sort_values("Time")

# ------------------------------------------------------------------
# Create plots (NOW 2 SUBPLOTS ONLY)
# ------------------------------------------------------------------
fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# ------------------------------------------------------------------
# Position Plot
# ------------------------------------------------------------------
axes[0].plot(df["Time"], df[body_columns["X"]], label="X")
axes[0].plot(df["Time"], df[body_columns["Y"]], label="Y")
axes[0].plot(df["Time"], df[body_columns["Z"]], label="Z")

axes[0].set_ylabel("Position")
axes[0].set_title(f"{body_name} Position vs Time")
axes[0].set_ylim(-1800, 2000)
axes[0].legend()
axes[0].grid(True)

# ------------------------------------------------------------------
# Orientation Plot
# ------------------------------------------------------------------
axes[1].plot(df["Time"], df[body_columns["Pitch"]], label="Pitch")
axes[1].plot(df["Time"], df[body_columns["Roll"]], label="Roll")
axes[1].plot(df["Time"], df[body_columns["Yaw"]], label="Yaw")

axes[1].set_ylim(-45, 40)
axes[1].set_ylabel("Angle (deg)")
axes[1].set_xlabel("Time (s)")
axes[1].set_title(f"{body_name} Orientation vs Time")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.show()