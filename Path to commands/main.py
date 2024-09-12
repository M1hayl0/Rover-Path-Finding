from path import GlobalPath, checkCoordinates
from commands import makeCommands
import numpy as np


def test(mapName, points):
    mapMatrix = np.load(f"{mapName}.npy")
    mapMatrixLayers = np.load(f"{mapName}Layers.npy")
    globalPath = GlobalPath(mapMatrix, mapMatrixLayers, 8, 0.167, 1, 0.05)
    pointsNames = ["S4", "W1", "W2", "W3", "W4", "W5", "W6", "W7", "W8", "W9"]

    for i in range(len(points)):
        start = points[i]
        if not checkCoordinates(*start, globalPath.mapMatrix, globalPath.roverDist):
            print("Wrong coordinates")
            break

        for j in range(len(points)):
            if i == j:
                continue

            end = points[j]
            if not checkCoordinates(*end, globalPath.mapMatrix, globalPath.roverDist):
                print("Wrong coordinates")
                break

            print(f"{pointsNames[i]}-{pointsNames[j]}")
            path = globalPath.aStar(start, end)
            print(path)
            commands = makeCommands(path, mapMatrix)
            print(commands)
            print(len(path), len(commands))
            print()


if __name__ == "__main__":
    newMapPoints = np.load("newMapPoints.npy")
    test("newMap", newMapPoints.tolist())
