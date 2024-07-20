import numpy as np

from common_functions import printTime, euclideanDistance, checkCoordinates


def minEuclideanDistance(point, path):
    minNode = path[0]
    minDist = np.inf
    for node in path:
        distance = euclideanDistance(point, node)
        if distance < minDist:
            minNode = node
            minDist = distance
    return minNode, minDist


def generateMatrix(matrixSize, probabilityForZero=0.7):
    matrix = np.random.choice([0, 1], size=(matrixSize[0], matrixSize[1]), p=[probabilityForZero, 1 - probabilityForZero])
    return matrix


class LocalPath:
    def __init__(self, globalPath, mapMatrix, steps, roverDist):
        self.globalPath = globalPath
        self.mapMatrix = generateMatrix([mapMatrix.shape[0], mapMatrix.shape[1]], 0.9)  # TODO: change this later when real input is provided by the rest of the team, this matrix should be changed every time algorithm is called
        self.steps = steps
        self.roverDist = roverDist

    def generatePath(self, roverPosition, roverOrientation, direction):
        directionRadians = np.radians(direction)
        dx = -np.cos(directionRadians)
        dy = np.sin(directionRadians)
        path = [roverPosition]
        turnAngle = 0
        if direction != roverOrientation:
            turnAngle = (direction - roverOrientation) / 5

        for _ in range(self.steps):
            newX = path[-1][0] + dx
            newY = path[-1][1] + dy
            newPosition = [newX, newY]
            if checkCoordinates(round(newPosition[0]), round(newPosition[1]), self.mapMatrix, self.roverDist):
                path.append(newPosition)

            direction += turnAngle
            directionRadians = np.radians(direction)
            dx = -np.cos(directionRadians)
            dy = np.sin(directionRadians)

        lastNode = None
        path2 = []
        for node in path:
            roundedNode = [round(node[0]), round(node[1])]
            if lastNode and lastNode[0] == roundedNode[0] and lastNode[1] == roundedNode[1]:
                continue
            path2.append(roundedNode)
            lastNode = roundedNode

        return path2, direction

    def generatePaths(self, roverPosition, roverOrientation, possibleDirections):
        possiblePaths = []
        for direction in possibleDirections:
            path, newDirection = self.generatePath(roverPosition, roverOrientation, direction)
            possiblePaths.append([path, newDirection])
        return possiblePaths

    def distanceFromGlobalPath(self, localPath):
        startNodeGlobalPath = minEuclideanDistance(localPath[0], self.globalPath)[0]
        startIndexGlobalPath = self.globalPath.index(startNodeGlobalPath)
        remainingGlobalPath = self.globalPath[startIndexGlobalPath:]

        distance = 0
        for node in localPath:
            distance += minEuclideanDistance(node, remainingGlobalPath)[1]
        distance /= len(localPath)

        return distance

    @printTime
    def dwa(self, roverPosition, roverOrientation):
        possibleDirections = [roverOrientation + d for d in [-40, -25, -10, 0, 10, 25, 40]]
        localPaths = self.generatePaths(roverPosition, roverOrientation, possibleDirections)
        localPathsCost = [0 for _ in range(len(localPaths))]

        for i in range(len(localPaths)):
            localPathsCost[i] += self.distanceFromGlobalPath(localPaths[i][0])
            for j in range(len(localPaths[i][0])):
                if self.mapMatrix[localPaths[i][0][j][0]][localPaths[i][0][j][1]] == 1:
                    localPathsCost[i] += 0.5  # TODO: change cost calculation after real input is provided

        chosenLocalPath = []
        minCost = np.inf
        for i in range(len(localPaths)):
            if localPathsCost[i] < minCost:
                minCost = localPathsCost[i]
                chosenLocalPath = localPaths[i]

        return chosenLocalPath
