import numpy as np
import queue
import time
import matplotlib.pyplot as plt


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


class GlobalPath:
    def __init__(self, mapMatrix, mapMatrixLayers, roverSize, distanceBetweenFields, roverMaxSlope, slopeThresholdIncrease):
        self.distanceBetweenFields = distanceBetweenFields  # in meters
        self.mapMatrix = mapMatrix
        self.mapMatrixLayers = mapMatrixLayers
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
        pq.put((0, start, [start]))  # cost, node, path
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
                        cost = len(path) + euclideanDistance([x, y], end)
                        addCost = 0
                        for point in self.mapMatrixLayers[x - self.roverDist:x + self.roverDist + 1, y - self.roverDist:y + self.roverDist + 1].reshape(-1):
                            addCost += cost * point / pow(self.roverSize, 2)
                        cost += addCost
                        pq.put((cost, [x, y], path + [[x, y]]))
                        visited.add((x, y))

    @printTime
    def aStar(self, start, end, searchDepth=4, initialSlopeThreshold=0.05):
        path = None
        currentSlopeThreshold = initialSlopeThreshold
        while currentSlopeThreshold < self.roverMaxSlope:
            path = self.aStarWithSlopeThreshold(start, end, searchDepth, currentSlopeThreshold)
            if path:
                return path
            currentSlopeThreshold += self.slopeThresholdIncrease
        return path


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

            path = globalPath.aStar(start, end)

            wholePath = []
            if path:
                for x, y in path:
                    for dx in range(-globalPath.roverDist, globalPath.roverDist + 1):
                        for dy in range(-globalPath.roverDist, globalPath.roverDist + 1):
                            wholePath.append([x + dx, y + dy])

            visualizePath(mapMatrix, wholePath, start, end, pointsNames[i], pointsNames[j])
            np.save(f"tests npy/{pointsNames[i]}-{pointsNames[j]}.npy", path)


if __name__ == "__main__":
    # test("oldMap", [[145, 80], [20, 10], [120, 20], [85, 30], [40, 80]])
    newMapPoints = np.load("newMapPoints.npy")
    test("newMap", newMapPoints.tolist())
