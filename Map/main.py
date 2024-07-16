import numpy as np
from plyfile import PlyData


def plyToArray(file):
    return np.array([[point[0], point[1], point[2]] for point in PlyData.read(file).elements[0].data])


def pointsArrayToMapMatrix(pointsArray):
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

    return mapMatrix


def saveMap(mapMatrix):
    np.savetxt("map.csv", mapMatrix, delimiter=",")
    np.save("map.npy", mapMatrix)


def main():
    pointsArray = plyToArray("72034.ply")
    mapMatrix = pointsArrayToMapMatrix(pointsArray)
    saveMap(mapMatrix)


if __name__ == "__main__":
    main()
