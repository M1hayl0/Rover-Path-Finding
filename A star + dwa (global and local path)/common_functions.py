import time
import numpy as np


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
