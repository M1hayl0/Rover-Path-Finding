import numpy as np
import queue
import cv2


def euclideanDistance(s, e):
    return np.linalg.norm(np.array(s) - np.array(e))


def generateMatrix(matrixSize, probabilityForZero=0.7):
    matrix = np.random.choice([0, 1], size=(matrixSize, matrixSize), p=[probabilityForZero, 1 - probabilityForZero])
    zeros = np.where(matrix == 0)
    start, end = np.random.randint(len(zeros[0]), size=2)
    start = [zeros[0][start], zeros[1][start]]
    end = [zeros[0][end], zeros[1][end]]
    matrix = matrix.tolist()
    return matrix, start, end


def aStar(matrix, start, end, matrixSize):
    pq = queue.PriorityQueue()
    # distance, node, path
    pq.put((0, start, [start]))
    matrix[start[0]][start[1]] = 2
    neighbours = [[-1, 0], [0, -1], [1, 0], [0, 1]]

    while not pq.empty():
        _, node, path = pq.get()
        if node == end:
            return path

        for neighbour in neighbours:
            x, y = node[0] + neighbour[0], node[1] + neighbour[1]
            if 0 <= x < matrixSize and 0 <= y < matrixSize and not matrix[x][y]:
                pq.put((len(path) + euclideanDistance([x, y], end), [x, y], path + [[x, y]]))
                matrix[x][y] = 2


def image(matrix, path, start, end):
    if path:
        for node in path:
            matrix[node[0]][node[1]] = [0, 255, 0]

    matrix[start[0]][start[1]] = [255, 0, 255]
    matrix[end[0]][end[1]] = [255, 165, 0]

    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0:
                matrix[i][j] = [255, 255, 255]
            elif matrix[i][j] == 1:
                matrix[i][j] = [0, 0, 0]
            elif matrix[i][j] == 2:
                matrix[i][j] = [255, 0, 0]

    matrix = np.array(matrix, dtype=np.uint8)
    img = cv2.cvtColor(matrix, cv2.COLOR_RGB2BGR)
    cv2.imwrite("result.png", img)


def main(matrixSize):
    matrix, start, end = generateMatrix(matrixSize)
    path = aStar(matrix, start, end, matrixSize)
    image(matrix, path, start, end)


if __name__ == "__main__":
    main(100)
