import numpy as np
import time

from global_path import GlobalPath
from local_path import LocalPath
from common_functions import euclideanDistance, checkCoordinates
import matplotlib.pyplot as plt


def visualizePath(matrix, path, start, end, startPoint, endPoint):
    plt.imshow(matrix, cmap="terrain", origin="upper")

    pathX, pathY = zip(*path)

    plt.plot(pathY, pathX, color="red", linestyle="-", linewidth=1, marker="o", markersize=3, markerfacecolor="blue", markeredgecolor="blue")
    plt.plot(start[1], start[0], marker="s", markersize=10, markerfacecolor="green", markeredgecolor="green")
    plt.plot(end[1], end[0], marker="s", markersize=10, markerfacecolor="red", markeredgecolor="red")

    plt.title(f"Global Path {startPoint}-{endPoint}")
    plt.xlabel("Column")
    plt.ylabel("Row")
    plt.colorbar(label="Terrain Height")
    plt.grid(visible=False)
    plt.savefig(f"tests/{startPoint}-{endPoint}.png")
    plt.show()


def main(mapName, points):
    mapMatrix = np.load(f"{mapName}.npy")
    globalPathObject = GlobalPath(mapMatrix, 8, 0.167, 1, 0.05)
    pointsNames = ["S4", "W1", "W2", "W3", "W4", "W5", "W6", "W7", "W8", "W9"]

    for i in range(len(points)):
        start = points[i]
        if not checkCoordinates(*start, globalPathObject.mapMatrix, globalPathObject.roverDist):
            print("Wrong coordinates")
            return

        for j in range(len(points)):
            if i == j:
                continue

            end = points[j]
            if not checkCoordinates(*end, globalPathObject.mapMatrix, globalPathObject.roverDist):
                print("Wrong coordinates")
                return

            path = globalPathObject.aStar(start, end)

            localPathObject = LocalPath(path, mapMatrix, 10, globalPathObject.roverDist)
            roverPosition = start

            roverOrientation = 0
            if path[1][0] == path[0][0] - 1 and path[1][1] == path[0][1]:
                roverOrientation = 0
            elif path[1][0] == path[0][0] - 1 and path[1][1] == path[0][1] + 1:
                roverOrientation = 45
            elif path[1][0] == path[0][0] and path[1][1] == path[0][1] + 1:
                roverOrientation = 90
            elif path[1][0] == path[0][0] + 1 and path[1][1] == path[0][1] + 1:
                roverOrientation = 135
            elif path[1][0] == path[0][0] + 1 and path[1][1] == path[0][1]:
                roverOrientation = 180
            elif path[1][0] == path[0][0] + 1 and path[1][1] == path[0][1] - 1:
                roverOrientation = 225
            elif path[1][0] == path[0][0] and path[1][1] == path[0][1] - 1:
                roverOrientation = 270
            elif path[1][0] == path[0][0] - 1 and path[1][1] == path[0][1] - 1:
                roverOrientation = 315

            wholePath = []
            while euclideanDistance(roverPosition, path[-1]) > 3:
                if euclideanDistance(roverPosition, path[-1]) < 15:
                    localPathObject.steps = 3
                elif euclideanDistance(roverPosition, path[-1]) < 10:
                    localPathObject.steps = 1

                localPath, roverOrientation = localPathObject.dwa(roverPosition, roverOrientation)
                roverPosition = localPath[-1]

                if localPath:
                    for x, y in localPath:
                        for dx in range(-globalPathObject.roverDist, globalPathObject.roverDist + 1):
                            for dy in range(-globalPathObject.roverDist, globalPathObject.roverDist + 1):
                                wholePath.append([x + dx, y + dy])

            visualizePath(mapMatrix, wholePath, start, end, pointsNames[i], pointsNames[j])


if __name__ == "__main__":
    newMapPoints = np.load("newMapPoints.npy")
    main("newMap", newMapPoints.tolist())
