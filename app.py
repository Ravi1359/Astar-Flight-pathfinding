import math
import heapq

def haversine(coord1, coord2):
    """
    Calculate the great-circle distance between two points 
    on the Earth's surface given their latitude and longitude.
    """
    R = 6371  # Radius of the Earth in kilometers
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    return distance

def astar(start, goal, neighbors_func):
    """
    Perform the A* algorithm to find the shortest path from start to goal.
    """
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    g_score = {start: 0}
    f_score = {start: haversine(start, goal)}
    came_from = {}
    
    visited = set()
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        if current in visited:
            continue
        
        visited.add(current)

        for neighbor in neighbors_func(current):
            current_to_goal = haversine(current, goal)
            neighbor_to_goal = haversine(neighbor, goal)
            
            # Allow moves that do not increase the distance to the goal
            if neighbor_to_goal > current_to_goal + 0.01:  # Adding a small epsilon to avoid floating-point issues
                continue
            
            tentative_g_score = g_score[current] + haversine(current, neighbor)
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + haversine(neighbor, goal)
                
                if neighbor not in visited:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # Return None if no path is found

def get_neighbors(coord):
    """
    Generate neighbors that are whole number coordinates.
    """
    lat, lon = coord
    lat_int, lon_int = int(lat), int(lon)
    
    # Define possible movements to whole number coordinates
    possible_moves = [
        (lat_int + 1, lon_int),
        (lat_int - 1, lon_int),
        (lat_int, lon_int + 1),
        (lat_int, lon_int - 1)
    ]
    
    # Filter out the moves that are not valid whole number coordinates
    neighbors = [(lat, lon) for lat, lon in possible_moves if isinstance(lat, int) and isinstance(lon, int)]
    
    return neighbors

# Example usage
if __name__ == "__main__":
    start = (37, -122)  # Using integer coordinates close to San Francisco
    goal = (34, -118)  # Using integer coordinates close to Los Angeles

    path = astar(start, goal, get_neighbors)
    if path:
        print("Path found:", path)
    else:
        print("No path found")
