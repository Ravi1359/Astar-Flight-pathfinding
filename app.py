import math
import heapq
from flask import Flask, request, jsonify

app = Flask(__name__)

def haversine(coord1, coord2):
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
            
            if neighbor_to_goal > current_to_goal + 0.01:
                continue
            
            tentative_g_score = g_score[current] + haversine(current, neighbor)
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + haversine(neighbor, goal)
                
                if neighbor not in visited:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None

def get_neighbors(coord):
    lat, lon = coord
    lat_int, lon_int = int(lat), int(lon)
    
    possible_moves = [
        (lat_int + 1, lon_int),
        (lat_int - 1, lon_int),
        (lat_int, lon_int + 1),
        (lat_int, lon_int - 1),
        (lat_int + 1, lon_int + 1),  # Diagonal north-east
        (lat_int + 1, lon_int - 1),  # Diagonal north-west
        (lat_int - 1, lon_int + 1),  # Diagonal south-east
        (lat_int - 1, lon_int - 1)   # Diagonal south-west
    ]
    
    neighbors = [(lat, lon) for lat, lon in possible_moves if isinstance(lat, int) and isinstance(lon, int)]
    
    return neighbors

@app.route('/path', methods=['POST'])
def find_path():
    data = request.get_json()
    start = tuple(data['start'])
    goal = tuple(data['goal'])
    
    path = astar(start, goal, get_neighbors)
    
    if path:
        return jsonify({'path': path}), 200
    else:
        return jsonify({'error': 'No path found'}), 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
