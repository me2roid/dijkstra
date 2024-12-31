import heapq
import numpy as np

def dijkstra(matrix, start_node):
    n = len(matrix)  # Number of nodes in the graph
    distances = [float('inf')] * n  # Initialize distances with infinity
    distances[start_node] = 0  # Distance to the start node is 0
    visited = [False] * n  # Track visited nodes
    priority_queue = [(0, start_node)]  # (distance, node)
    paths = {i: [] for i in range(n)}  # Dictionary to store paths
    paths[start_node] = [start_node]  # Start node path contains itself

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if visited[current_node]:
            continue

        visited[current_node] = True

        for neighbor in range(n):
            if matrix[current_node][neighbor] != float('inf') and not visited[neighbor]:
                new_distance = current_distance + matrix[current_node][neighbor]
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    heapq.heappush(priority_queue, (new_distance, neighbor))
                    paths[neighbor] = paths[current_node] + [neighbor]  # Update the path

    return distances, paths

# Define the adjacency matrix for the graph
adj_matrix = np.array([
    [0, 7, 12, float('inf'), float('inf'), float('inf')],  # Node 1
    [9, 0, 10, 11, 16, float('inf')],                     # Node 2
    [float('inf'), float('inf'), 0, float('inf'), 8, 6],  # Node 3
    [3, float('inf'), float('inf'), 0, 30, float('inf')], # Node 4
    [float('inf'), 18, 3, float('inf'), 0, 4],            # Node 5
    [float('inf'), float('inf'), 2, 4, 1, 0]              # Node 6
])

# Input start node from user
try:
    start_node = int(input("Enter the start node (1-6): ")) - 1  # Convert to 0-based index
    if start_node < 0 or start_node >= len(adj_matrix):
        raise ValueError("Invalid node number. Please enter a number between 1 and 6.")
    
    # Compute shortest paths and update the matrix
    shortest_paths, paths = dijkstra(adj_matrix, start_node)
    result_matrix = adj_matrix.copy()
    for i in range(len(shortest_paths)):
        result_matrix[start_node][i] = shortest_paths[i]  # Update the row with shortest paths

    print(f"Shortest paths from node {start_node + 1}:")
    print(np.array(result_matrix))  # Display the updated matrix

    print("\nPaths from start node to each node:")
    for node, path in paths.items():
        path_display = " -> ".join(str(p + 1) for p in path)  # Convert to 1-based indexing
        print(f"Node {start_node + 1} to Node {node + 1}: {path_display}")

except ValueError as e:
    print(e)