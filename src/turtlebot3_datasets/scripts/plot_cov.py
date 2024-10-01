import csv
import matplotlib.pyplot as plt
import numpy as np


class CSVPlotCov:
    def __init__(self, filepath):
        self.filepath = filepath
        self.amcl_cov = []
        self.amcl_error = []
        self.amcl_num_particles = []

    def load(self):
        with open(self.filepath, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)

            for row in reader:
                # Extract each column into its respective list
                if not (row[1] == ''):
                    self.amcl_cov.append([float(row[0]), float(row[1])])
                    self.amcl_num_particles.append([float(row[0]), float(row[3])])
                if not (row[2] == ''):
                    self.amcl_error.append([float(row[0]), float(row[2])])

            self.amcl_cov = np.array(self.amcl_cov)
            self.amcl_error = np.array(self.amcl_error)
            self.amcl_num_particles = np.array(self.amcl_num_particles)

    def plot(self):
        # Plot each column vs time
        plt.figure(figsize=(20, 13))

        plt.subplot(2, 1, 1)
        plt.step(self.amcl_cov[:, 0] - self.amcl_cov[0, 0], self.amcl_cov[:, 1], linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('Radius (m)')
        plt.ylim(0.2, 0.5)
        plt.title('Min. radius of a circle containing all particles')
        plt.grid()

        plt.subplot(2, 1, 2)
        plt.step(self.amcl_num_particles[:, 0] - self.amcl_num_particles[0, 0],
                 self.amcl_num_particles[:, 1], 'r', linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('Num Particles')
        plt.ylim(0, 1100)
        plt.title('Number of particles used by AMCL')
        plt.grid()
        plt.tight_layout()

        plt.figure(figsize=(10, 8))
        plt.plot(self.amcl_error[:, 0] - self.amcl_error[0, 0], self.amcl_error[:, 1], linewidth=3)
        plt.xlabel('Time (s)')
        plt.ylabel('Error (m)')
        plt.title('Position Estimation Error: AMCL vs Odom')
        plt.grid()

        plt.tight_layout()
        plt.show()


# Usage example
# Replace with your actual file path
extractor = CSVPlotCov('/home/rods/Desktop/IRob/plots/Amcl_cov_200_1000.csv')
extractor.load()
extractor.plot()
