import numpy as np
import cv2
import random
import queue
import copy


def generateMatrix(matrixSize, probabilityForZero=0.7):
    matrix = np.random.choice([0, 1], size=(matrixSize, matrixSize), p=[probabilityForZero, 1 - probabilityForZero])
    matrix = matrix.tolist()
    return matrix


def euclideanDistance(s, e):
    return np.linalg.norm(np.array(s) - np.array(e))


def minEuclideanDistance(point, path):
    minNode = path[0]
    minDist = np.inf
    for node in path:
        distance = euclideanDistance(point, node)
        if distance < minDist:
            minNode = node
            minDist = distance
    return minNode, minDist


def aStar(matrix, start, end, matrixSize):
    pq = queue.PriorityQueue()
    # distance, node, path
    pq.put((0, start, [start]))
    matrix_copy = copy.deepcopy(matrix)
    matrix_copy[start[0]][start[1]] = 2
    neighbours = [[-1, 0], [0, -1], [1, 0], [0, 1]]

    while not pq.empty():
        _, node, path = pq.get()
        if node == end:
            return path

        for neighbour in neighbours:
            x, y = node[0] + neighbour[0], node[1] + neighbour[1]
            if 0 <= x < matrixSize and 0 <= y < matrixSize and not matrix_copy[x][y]:
                pq.put((len(path) + euclideanDistance([x, y], end), [x, y], path + [[x, y]]))
                matrix_copy[x][y] = 2


class LocalPath:
    def __init__(self, globalPath, matrix, matrixSize, steps):
        self.globalPath = globalPath
        self.matrix = matrix
        self.matrixSize = matrixSize
        self.steps = steps

    def generatePath(self, roverPosition, roverOrientation, directionDegrees):
        directionRadians = np.radians(directionDegrees)
        dx = -np.cos(directionRadians)
        dy = np.sin(directionRadians)
        path = [roverPosition]
        turnAngle = 0
        if directionDegrees != roverOrientation:
            turnAngle = (directionDegrees - roverOrientation) / 5

        for _ in range(self.steps):
            newX = path[-1][0] + dx
            newY = path[-1][1] + dy
            newPosition = [newX, newY]
            if 0 < newPosition[0] < self.matrixSize - 1 and 0 < newPosition[1] < self.matrixSize - 1:
                path.append(newPosition)

            directionDegrees += turnAngle
            directionRadians = np.radians(directionDegrees)
            dx = -np.cos(directionRadians)
            dy = np.sin(directionRadians)

        lastNode = None
        path2 = []
        for node in path:
            roundedNode = [int(round(node[0])), int(round(node[1]))]
            if lastNode and lastNode[0] == roundedNode[0] and lastNode[1] == roundedNode[1]:
                continue
            path2.append(roundedNode)
            lastNode = roundedNode

        return path2

    def generatePaths(self, roverPosition, roverOrientation, possibleDirections):
        possiblePaths = []
        for direction in possibleDirections:
            path = self.generatePath(roverPosition, roverOrientation, direction)
            possiblePaths.append(path)
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

    def dwa(self, roverPosition, roverOrientation):
        possibleDirections = [roverOrientation + d for d in [-40, -25, -10, 0, 10, 25, 40]]
        localPaths = self.generatePaths(roverPosition, roverOrientation, possibleDirections)
        localPathsCost = [0 for _ in range(len(localPaths))]

        for i in range(len(localPaths)):
            localPathsCost[i] += self.distanceFromGlobalPath(localPaths[i])
            for j in range(len(localPaths[i])):
                if self.matrix[localPaths[i][j][0]][localPaths[i][j][1]] == 1:
                    localPathsCost[i] += 1

        chosenLocalPath = []
        minCost = np.inf
        for i in range(len(localPaths)):
            if localPathsCost[i] < minCost:
                minCost = localPathsCost[i]
                chosenLocalPath = localPaths[i]

        return localPaths, chosenLocalPath


def image(matrix, globalPath, localPaths, chosenLocalPath, roverPosition):
    if globalPath:
        for node in globalPath:
            matrix[node[0]][node[1]] = [255, 255, 0]

    if localPaths:
        for path in localPaths:
            for node in path:
                if matrix[node[0]][node[1]] == 1:
                    matrix[node[0]][node[1]] = [0, 130, 0]
                else:
                    matrix[node[0]][node[1]] = [0, 255, 0]

    if chosenLocalPath:
        for node in chosenLocalPath:
            if matrix[node[0]][node[1]] == [0, 130, 0]:
                matrix[node[0]][node[1]] = [0, 100, 100]
            else:
                matrix[node[0]][node[1]] = [0, 255, 255]

    matrix[roverPosition[0]][roverPosition[1]] = [255, 0, 0]

    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0:
                matrix[i][j] = [255, 255, 255]
            elif matrix[i][j] == 1:
                matrix[i][j] = [0, 0, 0]

    matrix = np.array(matrix, dtype=np.uint8)
    img = cv2.cvtColor(matrix, cv2.COLOR_RGB2BGR)
    cv2.imwrite("result.png", img)


def main(matrixSize):
    roverPosition = [int(matrixSize / 2), int(matrixSize / 2)]
    roverOrientation = random.randint(0, 360)
    matrix = generateMatrix(matrixSize, 0.9)
    globalPath = aStar(matrix, [2, 2], [matrixSize - 2, matrixSize - 2], matrixSize)
    localPath = LocalPath(globalPath, matrix, matrixSize, 10)
    if not globalPath:
        print("GLOBAL PATH NOT FOUND")
        return
    localPaths, chosenLocalPath = localPath.dwa(roverPosition, roverOrientation)
    image(matrix, globalPath, localPaths, chosenLocalPath, roverPosition)


if __name__ == "__main__":
    main(40)
