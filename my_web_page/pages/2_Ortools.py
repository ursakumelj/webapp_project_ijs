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

st.info("If the page fails to load properly, try refreshing it.")

st.write("OR-Tools is a powerful open-source software suite for optimization, developed by Google. The implementation of algorithm bellow is based on a paper https://medium.com/data-science/the-vehicle-routing-problem-exact-and-heuristic-solutions-c411c0f4d734. " \
"It expects a test problem in the CVRP problem format (see https://github.com/PyVRP/Instances/tree/main/CVRP). " \
"For solution strategies, CHRISTOFIDES and PATH_CHEAPEST_ARC are supported. Later it will be possible to choose more strategies. " \
"The solver gives you all the routes and final best cost, and visualizes them on a 2D plot.")

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
