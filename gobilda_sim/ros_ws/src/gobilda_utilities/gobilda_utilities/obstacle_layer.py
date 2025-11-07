import rclpy
from rclpy.node import Node

# Needed imports
from math import cos, sin, isfinite
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion


class LocalCostmap(Node):
    def __init__(self):
        super().__init__('local_costmap')

        # Map config
        self.map_width = 300            # cells
        self.map_height = 300           # cells
        self.map_resolution = 0.05      # resolution * cells  => 15 m x 15 m map

        # Occupancy conventions
        self.UNKNOWN = -1
        self.FREE = 0
        self.OCCUPIED = 100

        # Lidar Scan cache
        self.latest_scan = None

        # Outgoing OccupancyGrid message (we'll fill .info and .header)
        self.publish_map = OccupancyGrid()
        self._init_map_info()

        # Precompute robot's cell (center of grid)
        self.cx = self.map_width // 2
        self.cy = self.map_height // 2

    ''' Initialize static OccupancyGrid.info and origin so the robot is at the map center. '''
    def _init_map_info(self):
        self.publish_map.info.resolution = self.map_resolution
        self.publish_map.info.width = self.map_width
        self.publish_map.info.height = self.map_height

        # Place (0,0) of the grid so that the robot (base frame origin) is at the center cell.
        # That means the map origin (bottom-left corner in world coords) is shifted negative by half-size.
        origin = Pose()
        origin.position.x = - (self.map_width * self.map_resolution) / 2.0
        origin.position.y = - (self.map_height * self.map_resolution) / 2.0
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.publish_map.info.origin = origin

    ''' Meters in robot frame -> map indices (mx, my). Returns None if out of bounds. '''
    # Input: (x, y) coordinates of a point in the Cartesian plane
    # Output: Corresponding cell in the occupancy grid
    def world_to_map(self, x_m, y_m):
        # coordinate (0,0) is at bottom left of the map
        # Calculate meter distance from robot (at the center) and convert to cell index
        mx = int((x_m / self.map_resolution) + self.cx)
        my = int((y_m / self.map_resolution) + self.cy)
        if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
            return None, None
        return mx, my

    # Bresenham's Line Algorithm: inclusive endpoints
    # Input: 2-points on the Cartesian plane (i.e. a line)
    # (The first point is the robot origin, while the sencond is a single beam's endpoint)
    # Output: All the cells that that the beam crosses. i.e. the free cells.
    def bresenham_line_algorithm(self, x0, y0, x1, y1):
        freespace_cells = []
        
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        while(True):
            freespace_cells.append((x0, y0)) # add to empty
            e2 = 2 * err 
            if e2 >= dy: # if closer to next x cell
                if x0 == x1: break
                err += dy
                x0 += sx # step in x
            if e2 <= dx: # if closer
                if y0 == y1: break
                err += dx
                y0 += sy

        return freespace_cells
    
    ''' Cache the most recent LaserScan'''
    def laser_callback(self, msg: LaserScan):
        self.latest_scan = msg
        return

    # Input: x & y coordinates;
    # Output: list of free cells along the ray (excludes the last cell)
    def raytrace(self, x_cell, y_cell):
        # Compute free cells for a single beam
        # This function should call self.bresenham_line_algorithm
        return self.bresenham_line_algorithm(self.cx, self.cy, x_cell, y_cell)

    ''' Build and Publish the Occupancy Grid from the most recent LiDAR Scan '''
    def build_occupancy_grid(self):
        # First, check that the scan data is ready
        # Second, iterate through beams to create the map!

        if self.latest_scan:
            msg = self.latest_scan
            self.latest_scan = None  # clear cache
            
            current_angle = msg.angle_min
            for range in msg.ranges:
                if isfinite(range):
                    x_dist = range * cos(current_angle)
                    y_dist = range * sin(current_angle)
                    x_cell, y_cell = self.world_to_map(x_dist, y_dist)
                    free_cells = self.raytrace(x_cell, y_cell)
                current_angle += msg.angle_increment

        # Populate OccupancyGrid message
        self.publish_map.header.stamp = self.get_clock().now().to_msg()
        # Set frame to match your robot frame that LaserScan is in (commonly "base_link" or "laser")
        # The simulation and hardware will have different names for this frame
        self.publish_map.header.frame_id = 'diff_drive/lidar_link'
        # Publish
        self.publisher_.publish(self.publish_map)


def main(args=None):
    rclpy.init(args=args)
    
    # Node creation and spin
    local_costmap = LocalCostmap()
    rclpy.spin(local_costmap)
    
    # Node cleanup
    local_costmap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
