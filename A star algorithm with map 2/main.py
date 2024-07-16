import numpy as np
import queue
import time


def printTime(func):
    def wrapper(*args, **kwargs):
        startTime = time.time()
        result = func(*args, **kwargs)
        endTime = time.time()
        print(endTime - startTime)
        return result
    return wrapper


def euclideanDistance(s, e):
    return np.linalg.norm(np.array(s) - np.array(e))


def checkCoordinates(x, y, mapMatrix, roverDist):  # roverDist - distance from center to edge of the rover
    if (roverDist <= x < mapMatrix.shape[0] - roverDist and
            roverDist <= y < mapMatrix.shape[1] - roverDist and
            all([np.isfinite(z) for z in mapMatrix[x - roverDist:x + roverDist + 1, y - roverDist:y + roverDist + 1].reshape(-1)])):
        return True
    return False


class GlobalPath:
    def __init__(self, mapMatrix, roverSize, distanceBetweenFields, roverMaxSlope):
        self.distanceBetweenFields = distanceBetweenFields  # in meters
        self.mapMatrix = mapMatrix
        self.roverSize = roverSize if roverSize % 2 else roverSize + 1
        self.roverDist = (self.roverSize - 1) // 2  # distance from center to edge of the rover
        self.roverMaxSlope = roverMaxSlope

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

    @printTime
    def aStar(self, start, end, searchDepth=6):
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
                    pq.put(((len(path) + euclideanDistance([x, y], end)) + slope * (len(path) + euclideanDistance([x, y], end)), [x, y], path + [[x, y]]))
                    visited.add((x, y))


def test(points):
    mapMatrix = np.load("map.npy")
    globalPath = GlobalPath(mapMatrix, 4, 0.33, 1)

    for i in range(len(points)):
        start = points[i]
        if not checkCoordinates(*start, globalPath.mapMatrix, globalPath.roverDist):
            print("Wrong coordinates")
            break

        for j in range(i + 1, len(points)):
            end = points[j]
            if not checkCoordinates(*end, globalPath.mapMatrix, globalPath.roverDist):
                print("Wrong coordinates")
                break

            path = globalPath.aStar(start, end)

            mapMatrixWithPath = np.load("map.npy")
            if path:
                for x, y in path:
                    for dx in range(-globalPath.roverDist, globalPath.roverDist + 1):
                        for dy in range(-globalPath.roverDist, globalPath.roverDist + 1):
                            mapMatrixWithPath[x + dx, y + dy] = -np.inf

            np.savetxt(f"tests/map{i}{j}.csv", mapMatrixWithPath, delimiter=",")


if __name__ == "__main__":
    test([[145, 80], [20, 10], [120, 20], [85, 30], [40, 80]])
