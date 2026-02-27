"""
Grid utilities for converting workspace polygon to occupancy grid.
Used by the localization node to build /map for A* planning.
"""


def _point_in_polygon(px: float, py: float, polygon: list[tuple[float, float]]) -> bool:
    """
    Ray-casting point-in-polygon test.
    polygon: list of (x, y) vertices in CCW or CW order.
    """
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def polygon_to_occupancy_grid(
    polygon_xy: list[tuple[float, float]],
    resolution: float,
    inflation_radius: float = 0.0,
) -> tuple[list[list[int]], float, float]:
    """
    Build a 2D occupancy grid from a polygon (workspace boundary).

    Args:
        polygon_xy: List of (x, y) vertices in map frame (e.g. workspace_origin).
        resolution: Grid cell size in metres.
        inflation_radius: Extra cells to mark as occupied around the boundary (0 = no inflation).

    Returns:
        (grid_2d, origin_x, origin_y) where grid_2d is row-major [row][col],
        origin_x/origin_y are the world coordinates of grid cell (0,0).
        Values: 0 = free, 100 = occupied, -1 would indicate unknown (not used here).
    """
    if len(polygon_xy) < 3:
        return ([[100]], 0.0, 0.0)

    xs = [p[0] for p in polygon_xy]
    ys = [p[1] for p in polygon_xy]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Add margin for inflation
    margin = inflation_radius + resolution * 2
    min_x -= margin
    min_y -= margin
    max_x += margin
    max_y += margin

    origin_x = min_x
    origin_y = min_y

    width_m = max_x - min_x
    height_m = max_y - min_y
    cols = max(1, int(width_m / resolution) + 1)
    rows = max(1, int(height_m / resolution) + 1)

    grid = [[100] * cols for _ in range(rows)]

    for row in range(rows):
        for col in range(cols):
            wx = origin_x + (col + 0.5) * resolution
            wy = origin_y + (row + 0.5) * resolution
            if _point_in_polygon(wx, wy, polygon_xy):
                grid[row][col] = 0

    return (grid, origin_x, origin_y)


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> tuple[int, int]:
    """Convert world (m) to grid (col, row)."""
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return (col, row)


def grid_to_world(col: int, row: int, origin_x: float, origin_y: float, resolution: float) -> tuple[float, float]:
    """Convert grid (col, row) to world (x, y) at cell center."""
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return (x, y)
