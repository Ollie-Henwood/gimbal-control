import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------
file_path = "group9_fly_6D.tsv"

body1 = "group9_multirotor"
body2 = "camera"

# ------------------------------------------------------------------
# Load file + find header
# ------------------------------------------------------------------
with open(file_path, "r") as f:
    lines = f.readlines()

header_line_index = None
for i, line in enumerate(lines):
    if line.startswith("Frame\tTime"):
        header_line_index = i
        break

df = pd.read_csv(
    file_path,
    sep="\t",
    skiprows=header_line_index,
    engine="python"
)

df = df.dropna(axis=1, how="all")

# ------------------------------------------------------------------
# Extract body columns
# ------------------------------------------------------------------
def get_body_columns(columns, body_name):
    base = f"{body_name} X"
    i = columns.index(base)

    return {
        "X": columns[i],
        "Y": columns[i + 1],
        "Z": columns[i + 2],
        "Roll": columns[i + 3],
        "Pitch": columns[i + 4],
        "Yaw": columns[i + 5],
        "Residual": columns[i + 6],
    }

cols = list(df.columns)
b1 = get_body_columns(cols, body1)
b2 = get_body_columns(cols, body2)

# ------------------------------------------------------------------
# Numeric conversion
# ------------------------------------------------------------------
all_cols = ["Time"] + list(b1.values()) + list(b2.values())

for col in all_cols:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df = df.sort_values("Time")

# ------------------------------------------------------------------
# Reset time
# ------------------------------------------------------------------
df["Time"] = df["Time"] - df["Time"].iloc[0]

# ------------------------------------------------------------------
# Safe difference function
# If either value is 0 -> return NaN
# ------------------------------------------------------------------
def safe_diff(a, b):
    a = a.astype(float)
    b = b.astype(float)

    invalid = (a == 0) | (b == 0)
    return np.where(invalid, np.nan, b - a)

# ------------------------------------------------------------------
# Compute differences
# ------------------------------------------------------------------
diff = pd.DataFrame()
diff["Time"] = df["Time"]

for axis in ["X", "Y", "Z"]:
    diff[axis] = safe_diff(df[b1[axis]], df[b2[axis]])

for angle in ["Roll", "Pitch", "Yaw"]:
    diff[angle] = safe_diff(df[b1[angle]], df[b2[angle]])

# ------------------------------------------------------------------
# Plot
# ------------------------------------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(14, 14), sharex=True)

# Position
axes[0].plot(diff["Time"], diff["X"], label="ΔX")
axes[0].plot(diff["Time"], diff["Y"], label="ΔY")
axes[0].plot(diff["Time"], diff["Z"], label="ΔZ")
axes[0].set_title("Position Difference (camera - multirotor)")
axes[0].set_ylabel("millimeters")
axes[0].legend()
axes[0].grid(True)

# Orientation
axes[1].plot(diff["Time"], diff["Pitch"], label="ΔPitch")
axes[1].plot(diff["Time"], diff["Roll"], label="ΔRoll")
axes[1].plot(diff["Time"], diff["Yaw"], label="ΔYaw")
axes[1].set_title("Orientation Difference (camera - multirotor)")
axes[1].set_ylabel("degrees")
axes[1].legend()
axes[1].grid(True)

# Residuals
axes[2].plot(df["Time"], df[b1["Residual"]], label=f"{body1} residual")
axes[2].plot(df["Time"], df[b2["Residual"]], label=f"{body2} residual")

axes[2].set_title("Residuals")
axes[2].set_xlabel("Time (s)")
axes[2].set_ylabel("Residual value")
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()
plt.show()