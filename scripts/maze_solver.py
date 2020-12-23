from collections import deque
import numpy as np


# Function to check if it is possible to go to position (row, col)
# from current position. The function returns false if row, col
# is not a valid position or has value 0 or it is already visited
def isValid(mat, visited, row, col, M, N):
  return (row >= 0) and (row < M) and (col >= 0) and (col < N) \
       and mat[row][col] == 0 and not visited[row][col]
 
 
# Find Shortest Possible Route in a matrix mat from source
# cell (i, j) to destination cell (x, y)
def BFS(mat, source_x, source_y, dest_x, dest_y, M, N):
  # Below lists details all 4 possible movements from a cell
  row = [-1, 0, 0, 1]
  col = [0, -1, 1, 0]
 
  # construct a matrix to keep track of visited cells
  visited = [[False for x in range(N)] for y in range(M)]
  distances = [[0 for x in range(N)] for y in range(M)]
 
  # create an empty queue
  q = deque()
 
  # mark source cell as visited and enqueue the source node
  i, j = source_x, source_y
  visited[i][j] = True
 
  # (i, j, dist) represents matrix cell coordinates and its
  # minimum distance from the source
  q.append((i, j, 0))
  distances[i][j] = 0
 
  # stores length of longest path from source to destination
  min_dist = float('inf')
 
  # loop till queue is empty
  while q:
 
    # pop front node from queue and process it
    (i, j, dist) = q.popleft()
 
    # (i, j) represents current cell and dist stores its
    # minimum distance from the source
 
    # if destination is found, update min_dist and stop
    if i == dest_x and j == dest_y:
      min_dist = dist
      break
 
    # check for all 4 possible movements from current cell
    # and enqueue each valid movement
    for k in range(4):
      # check if it is possible to go to position
      # (i + row[k], j + col[k]) from current position
      if isValid(mat, visited, i + row[k], j + col[k], M, N):
        # mark next cell as visited and enqueue it
        visited[i + row[k]][j + col[k]] = True
        q.append((i + row[k], j + col[k], dist + 1))
        distances[i + row[k]][j + col[k]] = dist+1
 
  if min_dist == float('inf'):
    # print("Destination can't be reached from given source")
    return -1

  # print("The shortest path from source to destination has length", min_dist)
  # print(np.array(distances))
  # print("PATH:")
  dist = distances[i][j]
  path = []
  path.append((i,j))
  while dist!=1:
    # print("%d\t%d\t%d"%(i,j,dist))
    for k in range(4):
      if distances[i + row[k]][j + col[k]] == dist-1:
        i, j, dist = i + row[k], j + col[k], dist-1
        path.append((i,j))
        break
  path.append((source_x, source_y))
  # Reversing the list of points
  path = path[::-1]

  # Finding list of turns
  turns = []
  # I want prev point and next point to be sharing a corner, meaning I took a turn
  for i in range(1,len(path)-1):
    prev_pt = path[i-1]
    curr_pt = path[i]
    next_pt = path[i+1]
    if prev_pt[0]!=next_pt[0] and prev_pt[1]!=next_pt[1]:
      turns.append(curr_pt)
  turns.append((dest_x, dest_y))
  # print(turns)
  return turns[0]


# Shortest path in a Maze
# if __name__ == '__main__':

#   # input maze
#   mat = [
#     [1, 1, 1, 1, 1, 0, 0, 1, 1, 1],
#     [0, 1, 1, 1, 1, 1, 0, 1, 0, 1],
#     [0, 0, 1, 0, 1, 1, 1, 0, 0, 1],
#     [1, 0, 1, 1, 1, 0, 1, 1, 0, 1],
#     [0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
#     [1, 0, 1, 1, 1, 0, 0, 1, 1, 0],
#     [0, 0, 0, 0, 1, 0, 0, 1, 0, 1],
#     [0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
#     [1, 1, 1, 1, 1, 0, 0, 1, 1, 1],
#     [0, 0, 1, 0, 0, 1, 1, 0, 0, 1]
#   ]
 
#   # M x N matrix
#   M = N = 10
 
#   # Find shortest path from source (0, 0) to destination (7, 5)
#   BFS(mat, 0, 0, 7, 5, M, N)