#!/usr/bin/env python
import numpy as np
import os
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
from matplotlib.ticker import MaxNLocator

PATH = "/Users/margaridaestrela/Documents/tecnico/mestrado/irob/project/src/turtlebot3_datasets/scripts/Error_data"


class ErrorPlot:
    def __init__(self, Path, FileEndName, Title):
        self.DataGt = np.load(
            os.path.join(PATH, "ground_truth_data_" + FileEndName + ".npz")
        )
        self.DataEst = np.load(os.path.join(PATH, "EKF_data_" + FileEndName + ".npz"))
        self.Title = Title
        self.Time = self.DataEst["Time"] - self.DataEst["Time"][0]  # Start time at 0

        self.Translation = np.array([0.939, 1.275])

        Quat = np.array([0.001, -0.003, 0.738, 0.675])
        self.Rotation = R.from_quat(Quat).as_matrix()[:2, :2]
        # Interpolate the ground truth data to match the estimated data
        self.InterpolatePosition()
        self.TransformPosition()  # Transform the estimated data

    def EuclideanDistance(self):
        return np.linalg.norm(self.PosGt - self.PosEst, axis=0)

    def InterpolatePosition(self):
        TimeEst = self.DataEst["Time"]
        TimeGt = self.DataGt["Time"]

        PosXInterp = np.interp(TimeEst, TimeGt, self.DataGt["PosX"])
        PosYInterp = np.interp(TimeEst, TimeGt, self.DataGt["PosY"])

        self.PosGt = np.array([PosXInterp, PosYInterp])

    def TransformPosition(self):
        NewPos = []
        for x, y in zip(self.DataEst["PosX"], self.DataEst["PosY"]):
            OrigPos = np.array([x, y])
            NewPos.append(self.Rotation @ OrigPos + self.Translation)

        self.PosEst = np.array(NewPos).T

    def PlotDist(self):
        Error = self.EuclideanDistance()

        _, ax = plt.subplots(figsize=(10, 6))
        ax.plot(self.Time, Error, "r")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        # plt.title(
        #     "Position Estimation Error: Ground Truth vs EKF " + self.Title, fontsize=16
        # )
        plt.tight_layout()
        plt.grid()
        ax.text(
            0.95,
            0.95,
            f"Average Error: {np.mean(Error):.3f} m",
            transform=ax.transAxes,
            verticalalignment="top",
            horizontalalignment="right",
            bbox=dict(facecolor="white", alpha=0.5),
        )
        plt.savefig(os.path.join(PATH, "Error_" + self.Title + ".png"))

    def PlotCov(self):
        Position = self.PosEst
        Std = np.sqrt(np.vstack((self.DataEst["CovX"], self.DataEst["CovY"])))
        fig, ax = plt.subplots(2, 1, figsize=(10, 6))
        ax[0].plot(self.Time, Position[0], "o", label="Position X")
        ax[0].fill_between(
            self.Time,
            Position[0] - 2 * Std[0],
            Position[0] + 2 * Std[0],
            alpha=0.2,
            label="95% Confidence Interval",
        )
        ax[0].legend()
        ax[0].grid()
        ax[0].set_xlabel("Time (s)")
        ax[0].set_ylabel("Position (m)")

        ax[1].plot(self.Time, Position[1], "o", label="Position Y")
        ax[1].fill_between(
            self.Time,
            Position[1] - 2 * Std[1],
            Position[1] + 2 * Std[1],
            alpha=0.2,
            label="95% Confidence Interval",
        )
        ax[1].legend()
        ax[1].grid()

        ax[0].yaxis.set_major_locator(MaxNLocator(integer=True))
        ax[1].yaxis.set_major_locator(MaxNLocator(integer=True))

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        fig.subplots_adjust(wspace=0.2, hspace=0.2)
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.savefig(os.path.join(PATH, "Cov_" + self.Title + ".png"))
        # fig.suptitle("Estimated Position for EKF" + self.Title, fontsize=16)


if __name__ == "__main__":
    PlotNoGt = ErrorPlot(PATH, "NoGT", Title="(No GT Updates)")
    PlotNoGt.PlotDist()
    PlotNoGt.PlotCov()

    PlotComplete = ErrorPlot(PATH, "Complete", Title="(With GT Updates)")
    PlotComplete.PlotDist()
    PlotComplete.PlotCov()

    PlotOnlyGt = ErrorPlot(PATH, "OnlyGT", Title="(Only GT Updates)")
    PlotOnlyGt.PlotDist()
    PlotOnlyGt.PlotCov()

    PlotOnlyGt = ErrorPlot(
        PATH, "NewCov", Title="(Fine Tuned Covariance With GT Updates)"
    )
    PlotOnlyGt.PlotDist()
    PlotOnlyGt.PlotCov()

    plt.show()
