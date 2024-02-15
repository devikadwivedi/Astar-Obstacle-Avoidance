import heapq
import math
from shapely import Polygon, LineString

class State:
  def __init__(self, coordinates, g_value, h_value, parent_state):
    self.coordinates = coordinates
    self.g_value = g_value
    self.h_value = h_value
    self.f_value = g_value + h_value
    self.parent_state = parent_state

  def same_state(self, other):
    return (self.coordinates == other.coordinates
            and self.parent_state == other.parent_state)

  def __lt__(self, other):
    return self.f_value < other.f_value


# use an informed search: A* hueristic search
#shortest path from point A to a point C without hitting obstacles
# points have real coordinates and are VERTICES of the rectangular obstacles
# may move along the edge ofthe obstacle


# the A* algorithm
def search_algorithm(file_name):
  # read input file:
  file = open(file_name, 'r')
  line = file.readline().split()
  start = (int(line[0]), int(line[1]))

  line = file.readline().split()
  goal = (int(line[0]), int(line[1]))

  num_obstacles = int(file.readline())

  obstacles = []
  all_points = []
  for _ in range(num_obstacles):
    line = file.readline().split()
    obstacle = []
    obstacle.append((int(line[0]), int(line[1])))
    obstacle.append((int(line[2]), int(line[3])))
    obstacle.append((int(line[4]), int(line[5])))
    obstacle.append((int(line[6]), int(line[7])))
    all_points.append((int(line[0]), int(line[1])))
    all_points.append((int(line[2]), int(line[3])))
    all_points.append((int(line[4]), int(line[5])))
    all_points.append((int(line[6]), int(line[7])))
    poly = Polygon(obstacle)
    obstacles.append(poly)
  # create start state
  # compute f(s0) = g(s0)+h(s0) = h(s0)
  h_value = dist_func(start, goal)
  start_state = State(start, 0, h_value, None)
  open_list = []
  close_list = []

  # put [s0,f(s0)] in the open_list pq
  heapq.heappush(open_list, start_state)

  # if open_list is empty, output "Done" and stop
  while (open_list):
    # find and remove any item [s,p] on open_list having lowest p. break ties arbritarily
    curr_state = heapq.heappop(open_list)
    # put [s,p] on closed
    close_list.append(curr_state)
    # if s is the goal state, output its description (and backtrace a path). if h is known to be admissable, halt
    if curr_state.coordinates == goal:
      return backtrace_path(curr_state)

    # generate a list L of [s',f(s')] pairs where the s' are the successors of s and their f values are computed using f(s') = g(s')+h(s')
    L = generate_successors(curr_state, all_points, obstacles, goal)
    # consider each [s',f(s')]
    for successor in L:
      # if there is already a pair [s',q] on close for any value q
      for item in close_list:
        if (successor.same_state(item)):
          # if f(s') > q, then remove [s',f(s')] from L
          if (successor.f_value > item.f_value):
            L.remove(successor)
          # if f(s') <= q, then remove [s',q] from close
          if (successor.f_value <= item.f_value):
            close_list.remove(item)
      # else if there is already a pair [s',q] on open (for any value q)
      for item in open_list:
        if (successor.same_state(item)):
          # if f(s') > q, then remove [s',f(s')] from L
          if (successor.f_value > item.f_value):
            L.remove(successor)
          # if f(s') <= q, then remove [s',q] from open
          if (successor.f_value <= item.f_value):
            open_list.remove(item)
    # insert all members of L into open
    for successor in L:
      heapq.heappush(open_list, successor)

def generate_successors(curr_state, all_points, obstacles, goal):
  L = []
  # can go to any of the points given
  # invalid successor if it goes through an obstacle
  for point in all_points:
    # if the path is valid
    path = [curr_state.coordinates, point]
    if (is_valid(obstacles, path)):
      # g value is the distance from the start to current state
      g_value = curr_state.g_value + dist_func(curr_state.coordinates, point)
      # h value is the distance from the curent state to the goal
      h_value = dist_func(point, goal)
      # make the state
      new_state = State(point, g_value, h_value, curr_state)
      L.append(new_state)
  return L

def is_valid(obstacles, path):
  if (path[0] == path[1]):
    return False
  for obstacle in obstacles:
    line = LineString(path)
    if (line.crosses(obstacle) or obstacle.contains(line)):
      return False
  return True


def backtrace_path(curr_state):
  # backtrace states until there is None
  path = []
  cost = []
  while curr_state:
    path.append(curr_state.coordinates)
    cost.append(curr_state.g_value)
    curr_state = curr_state.parent_state

  path.reverse()
  cost.reverse()
  print("Point\tCumulative Cost")
  for i in range(len(path)):
    print(path[i], "\t", cost[i])

  return path


def dist_func(start_coord, dest_coord):
  # get straight-line distance between two points
  x1, y1 = start_coord
  x2, y2 = dest_coord
  return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)



search_algorithm("simple.txt")
