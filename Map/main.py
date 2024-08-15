import numpy as np
from plyfile import PlyData
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


def plyToArray(file):
    return np.array([[point[0], point[1], point[2]] for point in PlyData.read(file).elements[0].data])


def filterArray(pointsArray):
    tree = KDTree(pointsArray)
    radius = 3.0
    minNeighbors = 20

    filteredPoints = []
    for point in pointsArray:
        neighbors = tree.query_ball_point(point, radius)
        if len(neighbors) - 1 >= minNeighbors:
            filteredPoints.append(point)

    return np.array(filteredPoints)


def pointsArrayToMapMatrix(pointsArray, startEndPoints):
    xMin, xMax, yMin, yMax = np.min(pointsArray[:, 0]), np.max(pointsArray[:, 0]), np.min(pointsArray[:, 1]), np.max(pointsArray[:, 1])
    pointsDict = {}
    for point in pointsArray:
        x, y, z = point
        # -xMin to set the interval from 0, *3 so the map is more precise
        xy = (round((x - xMin) * 3), round((y - yMin) * 3))
        if xy in pointsDict:
            pointsDict[xy].append(z)
        else:
            pointsDict[xy] = [z]

    for key, value in pointsDict.items():
        # if more z coordinates have their x and y coordinates rounded to the same value take the average
        if len(value) > 1:
            pointsDict[key] = [sum(value)/len(value)]

    # the coordinates are in meters, so the spacing between fields in a matrix is going to be 0.33m because of multiplication with 3
    mapMatrix = np.full((round((xMax - xMin) * 3 + 1), round((yMax - yMin) * 3 + 1)), np.inf)

    zMin = min([value[0] for value in pointsDict.values()])
    for (x, y), z in pointsDict.items():
        mapMatrix[x, y] = z[0] - zMin

    for point in startEndPoints:
        point[0] = round((point[0] - xMin) * 3)
        point[1] = round((point[1] - yMin) * 3)

    return mapMatrix


def visualizeMap(matrix):
    plt.imshow(matrix, cmap='terrain', origin='upper')
    plt.title('Global Path')
    plt.xlabel('Column')
    plt.ylabel('Row')
    plt.colorbar(label='Terrain Height')
    plt.grid(visible=False)
    plt.show()


def main(mapName, startEndPoints):
    pointsArray = plyToArray(f"{mapName}.ply")
    if mapName == "newMap":
        pointsArray = filterArray(pointsArray)
    mapMatrix = pointsArrayToMapMatrix(pointsArray, startEndPoints)
    np.save(f"{mapName}.npy", mapMatrix)
    if mapName == "newMap":
        np.save(f"{mapName}Points.npy", startEndPoints)
    visualizeMap(mapMatrix)


if __name__ == "__main__":
    # main("oldMap", [])
    points = [
        [11.549, 17.228],  # S4
        [-2.259, 13.485],  # W1
        [10.337, 6.433],  # W2
        [7.58, 10.874],  # W3
        [6.71, 19.515],  # W4
        [4.851, 25.405],  # W5
        [0.211, 9.454],  # W6
        [0.83, 19.867],  # W7
        [9.082, 23.409],  # W8
        [3.435, 16.031],  # W9
    ]
    main("newMap", points)
