from path import euclideanDistance
import numpy as np


def makeCommands(path, mapMatrix):
    commandsList = []
    slopes = []
    currentDirection = [path[1][0] - path[0][0], path[1][1] - path[0][1]]
    currentLen = 1
    for i in range(len(path) - 2):
        if i % 2:
            continue
        totalRise = mapMatrix[path[i][0]][path[i][1]] - mapMatrix[path[i + 1][0]][path[i + 1][1]]
        totalRun = euclideanDistance(path[i], path[i + 1]) * 0.33
        slopes.append(np.arctan(totalRise / totalRun))

        newDirection = [path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]]
        if newDirection == currentDirection:
            currentLen += 1
        else:
            commandsList.append([currentDirection, currentLen, slopes])
            slopes = []
            currentDirection = newDirection
            currentLen = 1

    commandsList.append([currentDirection, currentLen, slopes])

    return commandsList
