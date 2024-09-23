#!/usr/bin/env python
import numpy as np
import os
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

PATH = '/home/rods/Desktop/IRob/src/turtlebot3_datasets/scripts/Error_Data'


def EuclideanDistance(Position1, Position2):
    return np.linalg.norm(Position1 - Position2, axis=0)


def InterpolatePosition(Time1, Time2, PosX, PosY):
    PosXInterp = np.interp(Time1, Time2, PosX)
    PosYInterp = np.interp(Time1, Time2, PosY)
    return np.array([PosXInterp, PosYInterp])


def PlotError(Time, Error):
    Time = Time - Time[0]  # Start time at 0
    _, ax = plt.subplots()
    ax.plot(Time, Error)
    ax.text(0.95, 0.95, f'Average Error: {np.mean(Error):.3f} m', transform=ax.transAxes,
            verticalalignment='top', horizontalalignment='right',
            bbox=dict(facecolor='white', alpha=0.5))
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.title("Error between Ground Truth and EKF")


def TransformPosition(PosX, PosY):
    Translation = np.array([0.939, 1.275])
    Quat = np.array([0.001, -0.003, 0.738, 0.675])
    Rotation = R.from_quat(Quat).as_matrix()[:2, :2]

    NewPos = []
    for x, y in zip(PosX, PosY):
        OrigPos = np.array([x, y])
        NewPos.append(Rotation @ OrigPos + Translation)

    return np.array(NewPos).T


def Plot(DataGt, DataEst):
    PosGt = InterpolatePosition(
        DataEst["Time"], DataGt["Time"], DataGt["PosX"], DataGt["PosY"])
    PosEst = TransformPosition(DataEst["PosX"], DataEst["PosY"])
    print(PosEst.shape)
    Error = EuclideanDistance(PosGt, PosEst)
    PlotError(DataEst["Time"], Error)


if __name__ == '__main__':
    Plot(np.load(os.path.join(PATH, 'ground_truth_data.npz')),
         np.load(os.path.join(PATH, 'odometry_data.npz')))

    Plot(np.load(os.path.join(PATH, 'ground_truth_dataNoGT.npz')),
         np.load(os.path.join(PATH, 'odometry_dataNoGT.npz')))

    Plot(np.load(os.path.join(PATH, 'ground_truth_dataOnlyGT.npz')),
         np.load(os.path.join(PATH, 'odometry_dataOnlyGT.npz')))

    plt.show()
