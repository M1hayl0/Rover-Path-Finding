import numpy as np
import queue

from common_functions import printTime, euclideanDistance, checkCoordinates


class GlobalPath:
    def __init__(self, mapMatrix, roverSize, distanceBetweenFields, roverMaxSlope, slopeThresholdIncrease):
        self.distanceBetweenFields = distanceBetweenFields  # in meters
        self.mapMatrix = mapMatrix
        self.roverSize = roverSize if roverSize % 2 else roverSize + 1
        self.roverDist = (self.roverSize - 1) // 2  # distance from center to edge of the rover
        self.roverMaxSlope = roverMaxSlope
        self.slopeThresholdIncrease = slopeThresholdIncrease

    def calculateSlope(self, center, neighbor, searchDepth):
        centerZ = self.mapMatrix[center[0]][center[1]]
        totalRise = 0
        totalRun = 0

        for dx in range(-searchDepth, searchDepth + 1):
            for dy in range(-searchDepth, searchDepth + 1):
                neighborX, neighborY = neighbor[0] + dx, neighbor[1] + dy
                if 0 <= neighborX < self.mapMatrix.shape[0] and 0 <= neighborY < self.mapMatrix.shape[1]:
                    neighborZ = self.mapMatrix[neighborX][neighborY]
                    if neighborZ != np.inf:
                        totalRise += neighborZ - centerZ
                        # z coordinates are in meters but spacing between fields in a matrix isn't exactly 1m
                        totalRun += euclideanDistance([neighborX, neighborY], center) * self.distanceBetweenFields

        if totalRun == 0:
            return 0
        slope = np.arctan(totalRise / totalRun)
        return slope

    def aStarWithSlopeThreshold(self, start, end, searchDepth, slopeThreshold):
        pq = queue.PriorityQueue()
        pq.put((0, start, [start]))  # distance, node, path
        visited = set()
        visited.add((start[0], start[1]))
        neighbours = [[-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]

        while not pq.empty():
            _, node, path = pq.get()
            if node == end:
                return path

            for neighbour in neighbours:
                x, y = node[0] + neighbour[0], node[1] + neighbour[1]
                if checkCoordinates(x, y, self.mapMatrix, self.roverDist) and (x, y) not in visited:
                    slope = self.calculateSlope(node, [x, y], searchDepth)
                    if abs(slope) < slopeThreshold:
                        pq.put((len(path) + euclideanDistance([x, y], end), [x, y], path + [[x, y]]))
                        visited.add((x, y))

    @printTime
    def aStar(self, start, end, searchDepth=6, initialSlopeThreshold=0.05):
        path = None
        currentSlopeThreshold = initialSlopeThreshold
        while currentSlopeThreshold < self.roverMaxSlope:
            path = self.aStarWithSlopeThreshold(start, end, searchDepth, currentSlopeThreshold)
            if path:
                return path
            currentSlopeThreshold += self.slopeThresholdIncrease
        return path
