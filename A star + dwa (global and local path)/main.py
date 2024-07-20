import numpy as np

from global_path import GlobalPath
from local_path import LocalPath
from common_functions import euclideanDistance, checkCoordinates


def main(start, end, roverOrientation):
    mapMatrix = np.load("map.npy")
    globalPathObject = GlobalPath(mapMatrix, 4, 0.33, 1, 0.05)

    if not checkCoordinates(*start, globalPathObject.mapMatrix, globalPathObject.roverDist):
        print("Wrong coordinates")
        return

    if not checkCoordinates(*end, globalPathObject.mapMatrix, globalPathObject.roverDist):
        print("Wrong coordinates")
        return

    globalPath = globalPathObject.aStar(start, end)

    mapMatrixWithPath = np.load("map.npy")
    if globalPath:
        for x, y in globalPath:
            for dx in range(-globalPathObject.roverDist, globalPathObject.roverDist + 1):
                for dy in range(-globalPathObject.roverDist, globalPathObject.roverDist + 1):
                    mapMatrixWithPath[x + dx, y + dy] = -np.inf
    np.savetxt(f"globalmap.csv", mapMatrixWithPath, delimiter=",")

    localPathObject = LocalPath(globalPath, mapMatrix, 10, globalPathObject.roverDist)
    roverPosition = start
    mapMatrixWithPath = np.load("map.npy")
    while euclideanDistance(roverPosition, globalPath[-1]) > 10:  # TODO: other way to end the journey
        localPath, roverOrientation = localPathObject.dwa(roverPosition, roverOrientation)
        roverPosition = localPath[-1]

        if localPath:
            for x, y in localPath:
                for dx in range(-globalPathObject.roverDist, globalPathObject.roverDist + 1):
                    for dy in range(-globalPathObject.roverDist, globalPathObject.roverDist + 1):
                        mapMatrixWithPath[x + dx, y + dy] = -np.inf

    np.savetxt(f"localmap.csv", mapMatrixWithPath, delimiter=",")


if __name__ == "__main__":
    main([120, 20], [40, 80], 45)  # TODO: rover orientation
    # main([20, 10], [85, 30], 180)
    # main([85, 30], [40, 80], 45)
    # main([20, 10], [40, 80], 90)
