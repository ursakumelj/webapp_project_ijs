# import streamlit as st
# from itertools import cycle
# import re
# import os
# import tempfile
# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# import matplotlib as mpl
# from ortools.constraint_solver import routing_enums_pb2
# from ortools.constraint_solver import pywrapcp
# from vrp_parser import parse_vrp_file 

# # -----------------------------
# # Helper functions
# # -----------------------------
# seed_prefix_map = {
#     "CHRISTOFIDES": "Christofides",
#     "PATH_CHEAPEST_ARC": "PATH_CHEAPEST"
# }

# def get_base_vehicle_count(vrp_file) -> int:
#     """Extract base vehicle count (integer after '-k') from filename"""
#     filename = getattr(vrp_file, "name", str(vrp_file))
#     match = re.search(r'-k(\d+)', filename, re.IGNORECASE)
#     if not match:
#         raise ValueError(f"Could not extract vehicle count from filename: {filename}")
#     return int(match.group(1))

# def get_initializer(initializer_name: str):
#     """Convert string to OR-Tools FirstSolutionStrategy enum"""
#     name = initializer_name.strip().upper()
#     if not hasattr(routing_enums_pb2.FirstSolutionStrategy, name):
#         raise ValueError(f"Invalid initializer name: {initializer_name}")
#     return getattr(routing_enums_pb2.FirstSolutionStrategy, name)

# # -----------------------------
# # VRP Solver
# # -----------------------------
# def solve_vrp(vrp_instance, n_vehicles, initializer_str="PATH_CHEAPEST_ARC", time_limit=10):
#     N = len(vrp_instance.node_coords)
#     capacity = int(vrp_instance.capacity)

#     # Prepare demands
#     demands_list = [0] * N
#     for node_id, demand_value in vrp_instance.demands.items():
#         index = vrp_instance.node_id_to_index[node_id]
#         demands_list[index] = int(demand_value)
#     demands = list(map(int, demands_list))

#     # Prepare distances
#     distances = np.round(np.array(vrp_instance.distance_matrix), decimals=4)

#     # OR-Tools setup
#     depot_node_index = vrp_instance.node_id_to_index[vrp_instance.depot_index]
#     manager = pywrapcp.RoutingIndexManager(N, n_vehicles, depot_node_index)
#     routing = pywrapcp.RoutingModel(manager)

#     def distance_callback(from_index, to_index):
#         return int(distances[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])

#     transit_callback_index = routing.RegisterTransitCallback(distance_callback)

#     def demand_callback(from_index):
#         return int(demands[manager.IndexToNode(from_index)])

#     demands_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

#     routing.AddDimensionWithVehicleCapacity(
#         demands_callback_index, 0, [capacity]*n_vehicles, True, 'Capacity'
#     )
#     routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

#     # Search parameters
#     search_parameters = pywrapcp.DefaultRoutingSearchParameters()
#     search_parameters.first_solution_strategy = get_initializer(initializer_str)
#     search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
#     search_parameters.time_limit.FromSeconds(time_limit)

#     solution = routing.SolveWithParameters(search_parameters)

#     solution = routing.SolveWithParameters(search_parameters)

#     if solution is not None:  
#         tours = []
#         total_distance = 0
#         for vehicle_id in range(n_vehicles):
#             index = routing.Start(vehicle_id)
#             tours.append([])
#             route_distance = 0
#             while not routing.IsEnd(index):
#                 node_index = manager.IndexToNode(index)
#                 tours[-1].append(node_index)
#                 previous_index = index
#                 index = solution.Value(routing.NextVar(index))
#                 route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
#             node_index = manager.IndexToNode(index)
#             tours[-1].append(node_index)
#             total_distance += route_distance
#         return total_distance, tours
#     else:
#         return None, None


# # -----------------------------
# # Plotting
# # -----------------------------
# def plot_routes_streamlit(vrp_instance, tours):
#     coordinates = pd.DataFrame([vrp_instance.node_coords[node_id] for node_id in sorted(vrp_instance.node_coords.keys())],
#                                columns=['x', 'y'])
#     cmap = mpl.colormaps["tab20"]
#     colors = cycle(cmap.colors)

#     fig, ax = plt.subplots(figsize=[8,6], dpi=100)
#     for r, tour in enumerate(tours):
#         if len(tour) > 1:
#             c = next(colors)
#             t = np.array(tour)
#             x = coordinates.values[t, 0]
#             y = coordinates.values[t, 1]
#             ax.scatter(x, y, color=c, label=f"Route {r+1}")
#             ax.plot(x, y, color=c)

#     depot_index = vrp_instance.node_id_to_index[vrp_instance.depot_index]
#     ax.scatter(coordinates.values[depot_index, 0], coordinates.values[depot_index, 1],
#                color='red', marker='s', s=100, label='Depot')
#     ax.legend(loc='upper left', bbox_to_anchor=(1, 1.02))
#     fig.tight_layout()
#     return fig

# # -----------------------------
# # Streamlit interface
# # -----------------------------
# st.set_page_config(page_title="VRP Solver", page_icon="🚚")
# st.title("🚚 Vehicle routing problem solver using OR-Tools")

# uploaded_file = st.file_uploader("Upload your VRP file (.vrp)", type=["vrp"])
# strategy = st.selectbox("Choose the initial solution strategy:", ["CHRISTOFIDES", "PATH_CHEAPEST_ARC"])

# if uploaded_file:
#     try:
#         # Step 1: Read uploaded file as string
#         file_content = uploaded_file.getvalue().decode("utf-8")
        
#         # Step 2: Parse VRP
#         vrp_instance = parse_vrp_file(file_content)
        
#         # Step 3: Get base vehicle count from filename
#         base_vehicles = get_base_vehicle_count(uploaded_file)
#         st.success(f"Detected base vehicle count: {base_vehicles}")
        
#         # Step 4: Solve VRP on button click
#         if st.button("Solve VRP"):
#             with st.spinner("Solving VRP..."):
#                 max_extra = 10
#                 found_solution = False
                
#                 for extra in range(max_extra + 1):
#                     n_vehicles = base_vehicles + extra
#                     distance, tours = solve_vrp(vrp_instance, n_vehicles, strategy, time_limit=30)
                    
#                     if tours is not None:
#                         found_solution = True
#                         st.success(f"Solved with {n_vehicles} vehicles! Total cost: {distance}")
                        
#                         # Show route details
#                         route_lines = []
#                         for i, route in enumerate(tours, start=1):
#                             route_nodes = [str(node) for node in route[1:-1]]
#                             if route_nodes:
#                                 route_lines.append(f"Route #{i}: {' '.join(route_nodes)}")
#                         route_lines.append(f"Cost {distance}")
#                         st.text("\n".join(route_lines))
                        
#                         # Plot routes
#                         fig = plot_routes_streamlit(vrp_instance, tours)
#                         st.pyplot(fig)
#                         break
#                 else:
#                     st.error(f"No feasible solution found with up to {base_vehicles + max_extra} vehicles.")
                    
#     except Exception as e:
#         st.error(f"VRP parser exception: {e}")

import streamlit as st
from itertools import cycle
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from vrp_parser import parse_vrp_file

# ============================================================
# SAFE LAZY IMPORT FOR OR-TOOLS (prevents segmentation faults)
# ============================================================

_routing_enums_pb2 = None
_pywrapcp = None
_ortools_ok = None


def ensure_ortools():
    """
    Safely try importing OR-Tools.
    Returns (ok: bool, message: str).
    Prevents hard crashes (segmentation faults).
    """
    global _routing_enums_pb2, _pywrapcp, _ortools_ok

    if _ortools_ok is not None:
        return _ortools_ok, "cached"

    try:
        import importlib
        _routing_enums_pb2 = importlib.import_module(
            "ortools.constraint_solver.routing_enums_pb2"
        )
        _pywrapcp = importlib.import_module(
            "ortools.constraint_solver.pywrapcp"
        )
        _ortools_ok = True
        return True, "ok"

    except Exception as e:
        _ortools_ok = False
        return False, str(e)


# ============================================================
# Helper Functions
# ============================================================

def get_base_vehicle_count(vrp_file) -> int:
    filename = getattr(vrp_file, "name", str(vrp_file))
    match = re.search(r'-k(\d+)', filename, re.IGNORECASE)
    if not match:
        raise ValueError(f"Could not extract vehicle count from filename: {filename}")
    return int(match.group(1))


def get_initializer(initializer_name: str):
    ok, msg = ensure_ortools()
    if not ok:
        raise RuntimeError(f"OR-Tools import failed: {msg}")

    name = initializer_name.strip().upper()
    if not hasattr(_routing_enums_pb2.FirstSolutionStrategy, name):
        raise ValueError(f"Invalid initializer name: {initializer_name}")

    return getattr(_routing_enums_pb2.FirstSolutionStrategy, name)


# ============================================================
# VRP Solver (using lazy-imported OR-Tools)
# ============================================================

def solve_vrp(vrp_instance, n_vehicles, initializer_str="PATH_CHEAPEST_ARC", time_limit=10):
    ok, msg = ensure_ortools()
    if not ok:
        raise RuntimeError(f"OR-Tools failed to load: {msg}")

    N = len(vrp_instance.node_coords)
    capacity = int(vrp_instance.capacity)

    # demands
    demands_list = [0] * N
    for node_id, demand_value in vrp_instance.demands.items():
        index = vrp_instance.node_id_to_index[node_id]
        demands_list[index] = int(demand_value)

    demands = list(map(int, demands_list))

    # distances
    distances = np.round(np.array(vrp_instance.distance_matrix), 4)

    depot_index = vrp_instance.node_id_to_index[vrp_instance.depot_index]

    manager = _pywrapcp.RoutingIndexManager(N, n_vehicles, depot_index)
    routing = _pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return int(distances[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])

    transit_id = routing.RegisterTransitCallback(distance_callback)

    def demand_callback(from_index):
        return int(demands[manager.IndexToNode(from_index)])

    demand_id = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_id, 0, [capacity] * n_vehicles, True, "Capacity"
    )
    routing.SetArcCostEvaluatorOfAllVehicles(transit_id)

    params = _pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = get_initializer(initializer_str)
    params.local_search_metaheuristic = (
        _routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    params.time_limit.FromSeconds(time_limit)

    solution = routing.SolveWithParameters(params)

    if solution is None:
        return None, None

    # extract solution
    tours = []
    total_cost = 0
    for v in range(n_vehicles):
        index = routing.Start(v)
        route = []
        route_cost = 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            prev = index
            index = solution.Value(routing.NextVar(index))
            route_cost += routing.GetArcCostForVehicle(prev, index, v)

        route.append(manager.IndexToNode(index))
        tours.append(route)
        total_cost += route_cost

    return total_cost, tours


# ============================================================
# Plotting
# ============================================================

def plot_routes_streamlit(vrp_instance, tours):
    coordinates = pd.DataFrame(
        [vrp_instance.node_coords[n] for n in sorted(vrp_instance.node_coords)],
        columns=["x", "y"]
    )

    cmap = mpl.colormaps["tab20"]
    colors = cycle(cmap.colors)

    fig, ax = plt.subplots(figsize=[8, 6], dpi=100)

    for i, tour in enumerate(tours):
        if len(tour) < 2:
            continue
        c = next(colors)
        t = np.array(tour)
        x = coordinates.values[t, 0]
        y = coordinates.values[t, 1]
        ax.scatter(x, y, color=c)
        ax.plot(x, y, color=c, label=f"Route {i+1}")

    depot_index = vrp_instance.node_id_to_index[vrp_instance.depot_index]
    ax.scatter(
        coordinates.values[depot_index, 0],
        coordinates.values[depot_index, 1],
        color="red",
        s=100,
        marker="s",
        label="Depot"
    )

    ax.legend(loc="upper left", bbox_to_anchor=(1, 1.02))
    fig.tight_layout()
    return fig


# ============================================================
# Streamlit UI
# ============================================================

st.set_page_config(page_title="VRP Solver", page_icon="🚚")
st.title("🚚 Vehicle Routing Problem Solver (OR-Tools)")

# check OR-Tools at startup
ok, msg = ensure_ortools()
if not ok:
    st.error(
        "❌ OR-Tools could not be loaded.\n"
        f"Error: **{msg}**\n\n"
        "This usually happens when OR-Tools is not compatible with your Python version.\n\n"
        "**Python 3.12 should work**, but if this continues, try Python 3.11."
    )

uploaded_file = st.file_uploader("Upload a VRP file (.vrp)", type=["vrp"])
strategy = st.selectbox(
    "Initial solution strategy:",
    ["CHRISTOFIDES", "PATH_CHEAPEST_ARC"]
)

if uploaded_file:
    try:
        text = uploaded_file.getvalue().decode("utf-8")
        vrp = parse_vrp_file(text)

        base_k = get_base_vehicle_count(uploaded_file)
        st.success(f"Base vehicle count: {base_k}")

        if st.button("Solve VRP"):
            with st.spinner("Solving..."):
                for extra in range(11):
                    k = base_k + extra
                    cost, tours = solve_vrp(vrp, k, strategy, time_limit=30)

                    if tours is not None:
                        st.success(f"Solution found with {k} vehicles. Cost: {cost}")

                        lines = []
                        for i, route in enumerate(tours, start=1):
                            nodes = [str(n) for n in route[1:-1]]
                            if nodes:
                                lines.append(f"Route {i}: {' '.join(nodes)}")
                        lines.append(f"Cost {cost}")

                        st.text("\n".join(lines))

                        fig = plot_routes_streamlit(vrp, tours)
                        st.pyplot(fig)
                        break
                else:
                    st.error("No feasible solution found.")

    except Exception as e:
        st.error(f"Error: {e}")
