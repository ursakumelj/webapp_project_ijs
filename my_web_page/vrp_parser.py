import re
from typing import List, Dict, Tuple

class VRPInstance:
    def __init__(self, name: str, capacity: int, node_coords: Dict[int, Tuple[float, float]], 
                 demands: Dict[int, float], depot_index: int = 1):
        self.name = name
        self.capacity = capacity
        self.node_coords = node_coords
        self.demands = demands
        self.depot_index = depot_index
        self.sorted_node_ids = sorted(node_coords.keys())
        self.node_id_to_index = {node_id: i for i, node_id in enumerate(self.sorted_node_ids)}
        self.distance_matrix = self._compute_distance_matrix()

    def _compute_distance_matrix(self) -> List[List[float]]:
        n = len(self.node_coords)
        matrix = [[0.0] * n for _ in range(n)]
        
        for id1, coord1 in self.node_coords.items():
            for id2, coord2 in self.node_coords.items():
                i = self.node_id_to_index[id1]
                j = self.node_id_to_index[id2]
                x1, y1 = coord1
                x2, y2 = coord2
                matrix[i][j] = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return matrix


def parse_vrp_file(content: str) -> VRPInstance:
    """Parse VRP file into more friendly format"""
    lines = content.split('\n')
    metadata = {}
    node_coords = {}
    demands = {}
    depot_index = 1  # Default depot index
    
    current_section = None
    for line in lines:
        line = line.strip()
        if not line:
            continue
            
        if line.startswith("NODE_COORD_SECTION"):
            current_section = "NODE_COORD"
            continue
        elif line.startswith("DEMAND_SECTION"):
            current_section = "DEMAND"
            continue
        elif line.startswith("DEPOT_SECTION"):
            current_section = "DEPOT"
            continue
        elif line.startswith("EOF"):
            break
            
        # {'NAME': 'X-n101-k25', ..., 'EDGE_WEIGHT_TYPE': 'EUC_2D', 'CAPACITY': '206'}
        if current_section is None and ":" in line:
            key, value = line.split(":", 1)
            key = key.strip().upper()
            value = value.strip()
            metadata[key] = value
            continue

        # {0: (365.0, 689.0), 1: (146.0, 180.0), ...}, (365.0, 689.0) are coordinates 
        if current_section == "NODE_COORD":
            parts = re.split(r'\s+', line.strip())
            if len(parts) >= 3:
                node_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                node_coords[node_id] = (x, y)
        
        # {0: 0.0, 1: 38.0, 2: 51.0, ...}, 0.0, 38.0, 51.0 are demands
        elif current_section == "DEMAND":
            parts = re.split(r'\s+', line.strip())
            if len(parts) >= 2:
                node_id = int(parts[0]) # IMPORTANT 1-based indexing
                demand = float(parts[1])
                demands[node_id] = demand
                
        elif current_section == "DEPOT":
            parts = re.split(r'\s+', line.strip())
            if parts[0].isdigit():
                depot_index = int(parts[0])
                break  # DEPOT_SECTION ends with -1

    return VRPInstance(
        name=metadata.get("NAME", "unknown"),
        capacity=float(metadata.get("CAPACITY", 0)),
        node_coords=node_coords,
        demands=demands,
        depot_index=depot_index
    )

def load_vrp_file(filepath: str) -> VRPInstance:
    """Load VRP instance from LOCAL file"""
    with open(filepath, 'r') as f:
        content = f.read()
    return parse_vrp_file(content)

### Test:
#instance = load_vrp_file("/Users/janze/Documents/ijs/webapp_project/X-n106-k14.vrp")
# print(f"Instance name: {instance.name}")
# print(f"Vehicle capacity: {instance.capacity}")
# print(f"Number of customers: {len(instance.demands) - 1}")  # minus depot
# print(f"Demands values: {instance.demands}")
# print(f"Distance matrix: {instance.distance_matrix[1][:]}")
# print(f"Distance between nodes 1 and 2: {instance.distance_matrix[0][1]}")
# print(f"Distance between nodes 1 and 2: {instance.distance_matrix[2][10]}")
# print(f"Distance between nodes 2 and 3: {instance.distance_matrix[10][13]}")
# print(f"Distance between nodes 3 and 1: {instance.distance_matrix[13][2]}")