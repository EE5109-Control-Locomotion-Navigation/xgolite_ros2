#!/usr/bin/env python3
"""
A* Planner Node
===============
Plans a path on the occupancy grid using 8-connected A*.
- Input: /map (OccupancyGrid), PlanPath service (start, goal)
- Output: /plan (Path) via service response
"""

import heapq
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan

from xgolite_nav.grid_utils import world_to_grid, grid_to_world


def _euclidean(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def astar(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
    rows: int,
    cols: int,
) -> list[tuple[int, int]]:
    """
    8-connected A*. Returns list of (col, row) from start to goal (inclusive).
    Occupied cells have value 100; free cells 0.
    """
    if start[0] < 0 or start[0] >= cols or start[1] < 0 or start[1] >= rows:
        return []
    if goal[0] < 0 or goal[0] >= cols or goal[1] < 0 or goal[1] >= rows:
        return []
    if grid[start[1]][start[0]] != 0:
        return []
    if grid[goal[1]][goal[0]] != 0:
        return []

    # 8-connected neighbours (col, row) - N, S, E, W, NE, NW, SE, SW
    dx = [0, 0, 1, -1, 1, -1, 1, -1]
    dy = [-1, 1, 0, 0, -1, -1, 1, 1]
    cost = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

    open_set: list[tuple[float, int, tuple[int, int]]] = []
    heapq.heappush(open_set, (0.0, 0, start))
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    counter = 1

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        cur_g = g_score[current]
        for i in range(8):
            nc = current[0] + dx[i]
            nr = current[1] + dy[i]
            if nc < 0 or nc >= cols or nr < 0 or nr >= rows:
                continue
            if grid[nr][nc] != 0:
                continue
            neighbor = (nc, nr)
            tentative_g = cur_g + cost[i]
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + _euclidean(neighbor, goal)
                counter += 1
                heapq.heappush(open_set, (f, counter, neighbor))

    return []


class AstarPlannerNode(Node):
    """A* path planner on occupancy grid."""

    def __init__(self):
        super().__init__('astar_planner_node')

        self.declare_parameter('map_frame', 'workspace_origin')

        self._map_frame = self.get_parameter('map_frame').value
        self._map: OccupancyGrid | None = None

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self._srv = self.create_service(GetPlan, '/plan_path', self._plan_cb)

        self.get_logger().info('A* planner ready.')

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _plan_cb(self, request: GetPlan.Request, response: GetPlan.Response):
        if self._map is None:
            self.get_logger().warn('No map received yet.')
            return response

        info = self._map.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution
        rows = info.height
        cols = info.width
        grid = [list(self._map.data[r * cols : (r + 1) * cols]) for r in range(rows)]

        start_x = request.start.pose.position.x
        start_y = request.start.pose.position.y
        goal_x = request.goal.pose.position.x
        goal_y = request.goal.pose.position.y

        sc, sr = world_to_grid(start_x, start_y, ox, oy, res)
        gc, gr = world_to_grid(goal_x, goal_y, ox, oy, res)

        def snap_to_free(c: int, r: int):
            if 0 <= c < cols and 0 <= r < rows and grid[r][c] == 0:
                return (c, r)
            for rad in range(1, max(rows, cols)):
                best = None
                best_d = float('inf')
                for dc in range(-rad, rad + 1):
                    for dr in range(-rad, rad + 1):
                        nc, nr = c + dc, r + dr
                        if 0 <= nc < cols and 0 <= nr < rows and grid[nr][nc] == 0:
                            d = dc * dc + dr * dr
                            if d < best_d:
                                best_d, best = d, (nc, nr)
                if best is not None:
                    return best
            return (c, r)

        sc, sr = snap_to_free(sc, sr)
        gc, gr = snap_to_free(gc, gr)

        path_grid = astar(grid, (sc, sr), (gc, gr), rows, cols)
        if not path_grid:
            self.get_logger().warn('A* could not find path.')
            return response

        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for col, row in path_grid:
            wx, wy = grid_to_world(col, row, ox, oy, res)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        response.plan = path_msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AstarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
