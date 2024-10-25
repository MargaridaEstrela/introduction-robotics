#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt
import os

# Load the CSV
file_path = "Error_data/amcl_200_1000.csv"
df = pd.read_csv(file_path)

df_clean = df.dropna(subset=["__time", "/amcl_error/data"])

# Extract the relevant columns
time = df_clean["__time"]
amcl_error = df_clean["/amcl_error/data"]

delta_time = time - time.iloc[0]

# Create the plot
fig = plt.figure(figsize=(10, 6))

# Plot error
plt.plot(delta_time, amcl_error, label="AMCL Error")

plt.xlabel("Time (s)")
plt.ylabel("Error (m)")


plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("AMCL_Error_200_1000.png")
plt.show()
