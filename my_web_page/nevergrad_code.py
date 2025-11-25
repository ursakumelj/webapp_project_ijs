"""
Nevergrad-only CVRP with fair decoding (optimal Split DP) + .sol warm-start

- Uses PyVRP ProblemData (same distances/demands/depot/capacity).
- Evaluates permutations with an OPTIMAL Split (DP) for distance+capacity.
- Warm-starts from .sol files (best-known or exact seeds), with ID normalization.
- Compare Nevergrad algorithms via `algo_name` ("NGOpt", "OnePlusOne", "DE", "CMA", "PSO"...).
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Sequence, Optional, Tuple
import numpy as np
from pyvrp import read
import nevergrad as ng

# -----------------------------
# Data model (PyVRP-backed)
# -----------------------------
@dataclass
class CVRPData:
    depot: int
    clients: List[int]     # client node ids (as in ProblemData)
    demand: np.ndarray     # demand indexed by node id (size = n_nodes)
    capacity: int          # homogeneous capacity
    dist: np.ndarray       # full distance matrix [n_nodes x n_nodes]

def _get_attr_any(obj, names: list[str]):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    raise AttributeError(f"None of {names} found on {type(obj)}")

def extract_cvrp_data(problem) -> CVRPData:
    """Extract essentials from PyVRP ProblemData, robust to version differences."""
    # Distance matrix
    D = None
    try:
        try:
            D = problem.distance_matrix(0)      # newer API (profile index)
        except TypeError:
            D = problem.distance_matrix()       # older API (no arg)
        except Exception:
            D = None
        if D is None:
            for prof in (1, 2, 3):
                try:
                    D = problem.distance_matrix(prof)
                    break
                except Exception:
                    pass
        if D is None and hasattr(problem, "distance_matrices"):
            mats = problem.distance_matrices()
            if isinstance(mats, (list, tuple)) and mats:
                D = mats[0]
        if D is None:
            raise RuntimeError("No distance matrix available via known APIs.")
    except Exception as e:
        raise RuntimeError("Could not obtain distance matrix from ProblemData.") from e

    D = np.asarray(D, dtype=float)
    n_nodes = int(D.shape[0])
    all_nodes = list(range(n_nodes))

    # Depot
    depot = None
    try:
        dep_obj = problem.depots()[0]
        depot = _get_attr_any(dep_obj, ["idx", "index", "id", "node"])
    except Exception:
        pass

    # Clients
    clients: List[int] = []
    try:
        for c in problem.clients():
            cid = None
            for nm in ("idx", "index", "id", "node"):
                if hasattr(c, nm):
                    cid = getattr(c, nm)
                    break
            if cid is None:
                raise AttributeError("Client missing id-like attribute")
            clients.append(int(cid))
    except Exception:
        if depot is not None and 0 <= int(depot) < n_nodes:
            clients = [i for i in all_nodes if i != int(depot)]
        else:
            depot = 0
            clients = list(range(1, n_nodes))

    if depot is None:
        rest = list(set(all_nodes) - set(clients))
        depot = int(rest[0]) if rest else 0

    # Demands (robust: .demand or .delivery)
    demand = np.zeros(n_nodes, dtype=int)
    try:
        for idx, c in enumerate(problem.clients()):
            cid = None
            for nm in ("idx", "index", "id", "node"):
                if hasattr(c, nm):
                    cid = getattr(c, nm)
                    break
            if cid is None and idx < len(clients):
                cid = clients[idx]
            if cid is None:
                continue
            val = None
            if hasattr(c, "demand"):
                val = getattr(c, "demand")
            elif hasattr(c, "delivery"):
                val = getattr(c, "delivery")
                if isinstance(val, (list, tuple)) and val:
                    val = val[0]
            if val is not None:
                try:
                    demand[int(cid)] = int(val)
                except Exception:
                    try:
                        demand[int(cid)] = int(float(val))
                    except Exception:
                        demand[int(cid)] = 0
    except Exception:
        pass

    # Capacity (prefer true value; fallbacks are last resort)
    capacity: Optional[int] = None
    for probe in (
        lambda: problem.vehicle_types()[0].capacity,
        lambda: problem.vehicleType(0).capacity(),
        lambda: getattr(problem, "capacity"),
        lambda: getattr(problem, "vehicle_capacity"),
    ):
        try:
            v = probe()
            if v is not None:
                capacity = int(v)
                break
        except Exception:
            pass
    if capacity is None or capacity <= 0:
        total_dem = int(demand.sum()) if demand is not None else 0
        max_dem = int(demand.max()) if demand is not None and demand.size > 0 else 1
        capacity = max(max_dem + 1, total_dem // 10 if total_dem > 0 else 50)

    return CVRPData(
        depot=int(depot),
        clients=[int(x) for x in clients],
        demand=demand,
        capacity=int(capacity),
        dist=D,
    )

def read_vrplib_capacity(vrp_path: str) -> int | None:
    """Read CAPACITY from VRPLIB header to override inconsistent API values."""
    try:
        with open(vrp_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if not s or ":" not in s:
                    if s.upper().startswith("NODE_COORD_SECTION"):
                        break
                    continue
                key, val = s.split(":", 1)
                if key.strip().upper() == "CAPACITY":
                    toks = val.replace("\t", " ").split()
                    for t in toks:
                        if t.isdigit():
                            return int(t)
                    try:
                        return int(float(val))
                    except Exception:
                        return None
    except Exception:
        return None
    return None

# -----------------------------
# Fair decoder: Optimal Split (DP) for distance + capacity
# -----------------------------
def split_opt_distance(
    perm: Sequence[int],
    demand: np.ndarray,
    Q: int,
    depot: int,
    D: np.ndarray,
) -> tuple[list[list[int]], float]:
    """
    Optimal Split for CVRP distance objective using DP (Vidal-style).
    Given a permutation 'perm' (client IDs), return (routes, total_cost).
    """
    n = len(perm)
    if n == 0:
        return [], 0.0

    # prefix loads
    load = [0] * (n + 1)
    for t in range(1, n + 1):
        load[t] = load[t - 1] + int(demand[perm[t - 1]])

    # cumulative inner-edge distances along the permutation (no depot legs)
    cumdist = [0.0] * (n + 1)
    for t in range(1, n):
        a = perm[t - 1]
        b = perm[t]
        cumdist[t + 1] = cumdist[t] + float(D[a, b])

    def route_cost(i: int, j: int) -> float:
        # cost of visiting perm[i..j], with depot legs
        cost = D[depot, perm[i]]
        cost += (cumdist[j + 1] - cumdist[i + 1])  # inner edges
        cost += D[perm[j], depot]
        return float(cost)

    INF = 1e100
    best = [INF] * (n + 1)
    prev = [-1] * (n + 1)
    best[0] = 0.0

    for t in range(1, n + 1):
        j = t - 1
        i = j
        while i >= 0:
            seg_load = load[j + 1] - load[i]
            if seg_load > Q:
                break
            cand = best[i] + route_cost(i, j)
            if cand < best[t]:
                best[t] = cand
                prev[t] = i
            i -= 1

    routes: list[list[int]] = []
    t = n
    while t > 0:
        i = prev[t]
        if i < 0:
            routes.append(list(perm[:t]))
            break
        routes.append(list(perm[i:t]))
        t = i
    routes.reverse()
    return routes, float(best[n])

# (Optional) simple greedy split (for ablations)
def split_capacity_only(perm: Sequence[int], demand: np.ndarray, Q: int) -> List[List[int]]:
    routes: List[List[int]] = []
    route: List[int] = []
    load = 0
    for cid in perm:
        d = int(demand[cid])
        if route and load + d > Q:
            routes.append(route)
            route, load = [], 0
        route.append(cid)
        load += d
    if route:
        routes.append(route)
    return routes

def route_length(route: Sequence[int], depot: int, D: np.ndarray) -> float:
    total, prev = 0.0, depot
    for cid in route:
        total += D[prev, cid]
        prev = cid
    return total + D[prev, depot]

def total_distance(routes: Sequence[Sequence[int]], depot: int, D: np.ndarray) -> float:
    return sum(route_length(r, depot, D) for r in routes)

# -----------------------------
# .sol helpers (seeds)
# -----------------------------
def parse_sol_file(path: str) -> tuple[list[list[int]], float | None]:
    """
    Parse CVRPLIB-like .sol file. Returns (routes, reported_cost).
    Lines like: "Route #1: 53 96 128" and "Cost 117595".
    """
    routes: list[list[int]] = []
    reported = None
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            L = s.lower()
            if L.startswith("route #"):
                try:
                    _, rhs = s.split(":", 1)
                    nodes = [int(tok) for tok in rhs.strip().split()]
                    routes.append(nodes)
                except Exception:
                    pass
            elif L.startswith("cost"):
                try:
                    toks = s.replace(":", " ").split()
                    for t in toks:
                        try:
                            reported = float(t)
                            break
                        except Exception:
                            continue
                except Exception:
                    reported = None
    return routes, reported

def routes_to_perm(routes: list[list[int]]) -> list[int]:
    return [cid for r in routes for cid in r]

def keys_from_perm_ids(data: CVRPData, perm_ids: list[int]) -> np.ndarray:
    """Build keys so that argsort(keys) reproduces perm_ids (aligned to data.clients)."""
    id_list = data.clients
    pos = {cid: i for i, cid in enumerate(perm_ids)}
    keys = np.empty(len(id_list), dtype=float)
    for i, cid in enumerate(id_list):
        keys[i] = pos[cid]
    return keys

def normalize_seed_perm_to_pyvrp_ids(data: CVRPData, perm_seed: list[int]) -> list[int] | None:
    """
    Convert a VRPLIB .sol permutation to PyVRP internal IDs and REPAIR it:
      - drop depot markers (1 and data.depot),
      - try identity, then minus-one shift,
      - remove duplicates (keep first occurrence),
      - append any missing client IDs (ascending) to complete a permutation.
    Returns a valid permutation (list of len == n_clients) or None if hopeless.
    """
    target_ids = list(sorted(data.clients))           # e.g. [1..218]
    target_set = set(target_ids)
    n = len(target_ids)

    # 0) strip obvious depot markers
    perm0 = [x for x in perm_seed if x not in (1, data.depot)]

    candidates = []

    # A) identity candidate
    candidates.append(perm0)

    # B) minus-one shift (typical VRPLIB client ids 2..(N) -> 1..(N-1))
    candidates.append([x - 1 for x in perm0])

    # C) generic shift by aligning minima (last resort)
    try:
        shift = min(target_ids) - min(perm0)
        candidates.append([x + shift for x in perm0])
    except ValueError:
        pass  # empty perm0 etc.

    # Try each candidate with repair
    for cand in candidates:
        # filter out anything not in plausible range (to avoid negatives etc.)
        cand = [int(x) for x in cand if isinstance(x, (int, np.integer))]
        # deduplicate while preserving order
        seen = set()
        dedup = []
        for x in cand:
            if x in target_set and x not in seen:
                seen.add(x)
                dedup.append(x)
        # append any missing clients (sorted) to make it a full permutation
        missing = [x for x in target_ids if x not in seen]
        repaired = dedup + missing
        if len(repaired) == n and set(repaired) == target_set:
            return repaired

    # give up
    return None
def _debug_check_seed(data: CVRPData, perm_seed_raw: list[int]):
    print("\n[debug] --- SEED DIAGNOSTICS ---")
    print(f"[debug] depot={data.depot}; clients(min..max)={min(data.clients)}..{max(data.clients)}; n={len(data.clients)}")
    print(f"[debug] seed_raw len={len(perm_seed_raw)}  head={perm_seed_raw[:10]} ... tail={perm_seed_raw[-10:]}")

    mapped = normalize_seed_perm_to_pyvrp_ids(data, perm_seed_raw)
    if mapped is None:
        print("[debug] mapping FAILED -> seed will be skipped")
        return None

    print(f"[debug] mapped len={len(mapped)}  head={mapped[:10]} ... tail={mapped[-10:]}")
    keys = keys_from_perm_ids(data, mapped)
    idx = np.argsort(keys)
    perm_rebuilt = [data.clients[i] for i in idx]
    ok = (perm_rebuilt == mapped)
    print(f"[debug] argsort(keys) reproduces mapped perm? {ok}")
    if not ok:
        # show first mismatch
        for i,(a,b) in enumerate(zip(perm_rebuilt, mapped)):
            if a != b:
                print(f"[debug] first mismatch at pos {i}: rebuilt={a}, mapped={b}")
                break
    return mapped, keys
# -----------------------------
# Nevergrad optimization (with logging + warm-start from .sol)
# -----------------------------
def run_nevergrad(
    data: CVRPData,
    budget: int,
    algo_name: str = "NGOpt",
    use_split_decoder: bool = True,
    log_every: int = 100,
    initial_sol_paths: list[str] | None = None,
):
    # 🔧 normalize seeds: allow str or list[str]
    if isinstance(initial_sol_paths, str):
        initial_sol_paths = [initial_sol_paths]
    """
    Runs a Nevergrad optimizer on the CVRP, optionally seeded with .sol solutions.
    Returns (best_cost, best_routes, history), where history = [(eval, best_cost, num_routes)].
    """
    n = len(data.clients)
    param = ng.p.Array(shape=(n,)).set_bounds(-1e-6, float(n) - 1e-6)

    OptimizerClass = getattr(ng.optimizers, algo_name)
    optimizer = OptimizerClass(parametrization=param, budget=budget, num_workers=1)

    id_list = data.clients

    def eval_perm_from_keys(keys_arr: np.ndarray) -> tuple[float, list[list[int]]]:
        idx = np.argsort(keys_arr)
        perm = [id_list[i] for i in idx]
        if use_split_decoder:
            routes, val = split_opt_distance(perm, data.demand, data.capacity, data.depot, data.dist)
            return float(val), routes
        # fallback: greedy split (for ablations)
        routes = split_capacity_only(perm, data.demand, data.capacity)
        return float(total_distance(routes, data.depot, data.dist)), routes

    # ------------------------------------------------------------------
# INITIALIZATION + WARM START FROM .SOL FILES (with proper best tracking)
# ------------------------------------------------------------------
    best_cost = float("inf")
    best_routes: list[list[int]] = []
    history: list[tuple[int, float, int]] = []

    if initial_sol_paths:
        seen = set()
        for spath in initial_sol_paths:
            try:
                routes_seed, _ = parse_sol_file(spath)
                perm_seed_raw = routes_to_perm(routes_seed)

                # DEBUG: show mapping + reproduction
                dbg = _debug_check_seed(data, perm_seed_raw)
                if dbg is None:
                    print(f"[seed skip] {spath}: could not map .sol IDs to PyVRP client IDs.")
                    continue
                mapped, keys = dbg

                # tiny jitter to avoid tie issues; but keep a copy for evaluation first
                keys_eval = keys.copy()
                # ensure uniqueness signature to avoid duplicate tells
                sig = tuple(np.argsort(keys_eval))
                if sig in seen:
                    print(f"[seed skip] {spath}: duplicate after mapping.")
                    continue
                seen.add(sig)

                # Evaluate with the SAME objective as the run
                cost, rts = eval_perm_from_keys(keys_eval)

                # Tell optimizer and set incumbent
                keys_seed = np.asarray(keys_eval, dtype=float)
                # deterministic micro-offset to break ties without leaving bounds
                keys_seed += np.linspace(0.0, 1e-9, len(keys_seed), dtype=float)
                # hard clip to bounds
                keys_seed = np.clip(keys_seed, 0.0, float(n) - 1e-6)
                child = optimizer.parametrization.spawn_child(new_value=keys_seed)
                optimizer.tell(child, cost)

                if cost < best_cost:
                    best_cost = cost
                    best_routes = rts

                print(f"[seeded] {spath} -> cost={cost:.3f}, routes={len(rts)}")
            except Exception as e:
                print(f"[seed error] {spath}: {e}")

        if best_cost < float("inf"):
            history.append((0, float(best_cost), len(best_routes)))
            print(f"[seed incumbent] best={best_cost:.3f} routes={len(best_routes)}")

    # ------------------------------------------------------------------
    # MAIN OPTIMIZATION LOOP
    # ------------------------------------------------------------------
    for k in range(1, budget + 1):
        cand = optimizer.ask()
        arr = cand.value if hasattr(cand, "value") else cand
        keys = np.asarray(arr, dtype=float).ravel()

        cost, routes = eval_perm_from_keys(keys)
        optimizer.tell(cand, cost)

        if cost < best_cost:
            best_cost = cost
            best_routes = routes

        if log_every and (k % log_every == 0):
            print(f"[{k:>6d}/{budget}] best={best_cost:.3f}  routes={len(best_routes)}")
            history.append((k, float(best_cost), len(best_routes)))

    # Final recommendation check
    rec = optimizer.provide_recommendation()
    rarr = rec.value if hasattr(rec, "value") else rec
    rcost, rroutes = eval_perm_from_keys(np.asarray(rarr, float).ravel())
    if rcost < best_cost:
        best_cost, best_routes = rcost, rroutes

    return float(best_cost), best_routes, history

# -----------------------------
# Experiment convenience wrapper
# -----------------------------
def experiment(
    vrp_path: str,
    budget: int = 2000,
    algo_name: str = "NGOpt",
    initial_sol_paths: list[str] | None = None,
    use_split_decoder: bool = True,
    log_every: int = 100,
):
    pdata = read(vrp_path)
    data = extract_cvrp_data(pdata)

    # enforce header CAPACITY if it disagrees with ProblemData (important for X-n219-k73)
    cap_hdr = read_vrplib_capacity(vrp_path)
    if cap_hdr is not None and cap_hdr != data.capacity:
        print(f"[warn] PyVRP reported capacity {data.capacity}, but VRPLIB header says {cap_hdr}. Using header value.")
        data.capacity = int(cap_hdr)

    print("Instance loaded:")
    print(f"  Depot:      {data.depot}")
    print(f"  #Clients:   {len(data.clients)}")
    print(f"  Capacity:   {data.capacity}")
    print(f"  Total dem.: {int(data.demand.sum())} (max per client: {int(data.demand.max())})")

    print("\nRunning Nevergrad optimization...")
    best_cost, best_routes, hist = run_nevergrad(
        data=data,
        budget=budget,
        algo_name=algo_name,
        use_split_decoder=use_split_decoder,
        log_every=log_every,
        initial_sol_paths=initial_sol_paths,
    )

    print(f"\nNevergrad best cost: {best_cost:.3f}")
    print(f"#Routes: {len(best_routes)}")
    if hist:
        e, c, r = hist[-1]
        print(f"Last checkpoint @ {e} evals: best={c:.3f}, routes={r}")

    return {"best_cost": best_cost, "routes": best_routes, "history": hist}
# ---------- helpers for absolute-cost comparisons ----------
def cost_of_routes_on_matrix(routes: list[list[int]], depot: int, D: np.ndarray) -> float:
    tot = 0.0
    for r in routes:
        prev = depot
        for cid in r:
            tot += float(D[prev, cid])
            prev = cid
        tot += float(D[prev, depot])
    return float(tot)

def reevaluate_sol(vrp_path: str, sol_path: str):
    """Loads instance & .sol, returns (abs_cost_on_our_matrix, n_routes, data)."""
    pdata = read(vrp_path)
    data = extract_cvrp_data(pdata)
    # keep your header override for capacity, to be consistent
    cap_hdr = read_vrplib_capacity(vrp_path)
    if cap_hdr is not None and cap_hdr != data.capacity:
        data.capacity = int(cap_hdr)
    routes_seed, reported = parse_sol_file(sol_path)
    cost_abs = cost_of_routes_on_matrix(routes_seed, data.depot, data.dist)
    return {"cost_abs": cost_abs, "routes": len(routes_seed), "reported": reported, "data": data}
def sol_perm_split_cost(
    vrp_path: str,
    sol_path: str,
) -> tuple[float, int, list[list[int]]]:
    """
    Re-evaluate a .sol 'as Nevergrad sees it':
    1) load instance (PyVRP),
    2) build the SAME distance matrix you use now,
    3) parse .sol routes -> flatten to a giant tour,
    4) normalize/repair IDs -> PyVRP client ids,
    5) run the SAME Split DP decoder,
    6) return (cost, #routes, routes_after_split).
    """
    pdata = read(vrp_path)
    data = extract_cvrp_data(pdata)

    # keep your header override for capacity (e.g., X-n219-k73: CAPACITY 3)
    cap_hdr = read_vrplib_capacity(vrp_path)
    if cap_hdr is not None and cap_hdr != data.capacity:
        data.capacity = int(cap_hdr)

    # (optional) if you rebuilt EUC_2D integer matrix elsewhere, apply it here too
    # data.dist = rebuild_vrplib_euc2d_matrix(vrp_path)

    routes_seed, _reported = parse_sol_file(sol_path)
    perm_raw = [cid for r in routes_seed for cid in r]

    # map/repair to PyVRP ids (drop depot, minus-one shift, de-dup, append missing)
    mapped = normalize_seed_perm_to_pyvrp_ids(data, perm_raw)
    if mapped is None:
        raise ValueError("Could not map .sol IDs to PyVRP client IDs for perm+split.")

    # run the SAME decoder Nevergrad uses
    routes_split, cost_split = split_opt_distance(
        mapped, data.demand, data.capacity, data.depot, data.dist
    )
    return float(cost_split), len(routes_split), routes_split
def run_suite_compare(
    vrp_path: str,
    algos: list[str],
    budget: int,
    sol_path: str | None = None,
    log_every: int = 0,                 # silence inner logs by default
    use_split_decoder: bool = True,     # fair decoding (optimal Split DP)
):
    """
    Runs each algo twice: (A) unseeded, (B) seeded with sol_path (if given).
    Returns list of dicts with absolute costs and gaps to the reevaluated .sol.
    """
    # 1) Reference: .sol re-evaluated with perm + Split (Nevergrad-equivalent)
    ref = None
    if sol_path is not None:
        ref_cost, ref_routes, _ = sol_perm_split_cost(vrp_path, sol_path)
        ref = {"cost_abs": ref_cost, "routes": ref_routes, "reported": None}
        # (you can still compute and print the exact .sol routes cost if you want)

    # 2) Prepare shared data once (same matrix & representation for all algos)
    pdata = read(vrp_path)
    data = extract_cvrp_data(pdata)
    cap_hdr = read_vrplib_capacity(vrp_path)
    if cap_hdr is not None and cap_hdr != data.capacity:
        data.capacity = int(cap_hdr)

    results = []
    for algo in algos:
        # A) Unseeded
        best_cost_A, best_routes_A, _hist_A = run_nevergrad(
            data=data,
            budget=budget,
            algo_name=algo,
            use_split_decoder=use_split_decoder,
            log_every=log_every,
            initial_sol_paths=None,
        )
        entryA = {
            "algo": algo,
            "mode": "unseeded",
            "cost": float(best_cost_A),
            "routes": int(len(best_routes_A)),
        }
        if ref is not None:
            entryA["gap_%"] = 100.0 * (entryA["cost"] - ref_cost) / ref_cost
        results.append(entryA)

        # B) Seeded (if .sol provided)
        if sol_path is not None:
            best_cost_B, best_routes_B, _hist_B = run_nevergrad(
                data=data,
                budget=budget,
                algo_name=algo,
                use_split_decoder=use_split_decoder,
                log_every=log_every,
                initial_sol_paths=[sol_path],   # seed here
            )
            entryB = {
                "algo": algo,
                "mode": "seeded",
                "cost": float(best_cost_B),
                "routes": int(len(best_routes_B)),
            }
            if ref is not None:
                entryB["gap_%"] = 100.0 * (entryB["cost"] - ref_cost) / ref_cost
            results.append(entryB)

    # 3) Pretty print summary
    print("\n=== SUMMARY (absolute numbers on current matrix) ===")
    if ref is not None:
        rep = f"{ref['reported']:.3f}" if isinstance(ref["reported"], (int, float)) else str(ref["reported"])
        print(f"Re-evaluated .sol:  cost={ref_cost:.3f}  routes={ref_routes}  (file says: {rep})")
    header = f"{'Algo':12} {'Mode':9} {'Cost':>12} {'Routes':>8}" + (f" {'Gap% vs .sol':>12}" if ref is not None else "")
    print(header)
    print("-" * len(header))
    for r in results:
        line = f"{r['algo']:12} {r['mode']:9} {r['cost']:12.3f} {r['routes']:8d}"
        if ref is not None:
            line += f" {r['gap_%']:12.3f}"
        print(line)

    return {"reference": ref, "results": results}


import time, csv, random, os
import numpy as np
from pathlib import Path

# -------------------------------------------------
# 1. Helpers
# -------------------------------------------------
def set_all_seeds(seed: int):
    random.seed(seed)
    np.random.seed(seed)

def infer_sol_sibling_path(vrp_path: str, sol_path: str | None) -> str | None:
    """If sol_path is None, use <basename>.sol next to the .vrp."""
    if sol_path:
        return sol_path
    p = Path(vrp_path)
    guess = p.with_suffix(".sol")
    return str(guess) if guess.exists() else None

def save_sol_file(path: str, routes: list[list[int]], cost: float | int):
    """Writes solution in standard CVRPLIB .sol format."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for i, r in enumerate(routes, 1):
            f.write(f"Route #{i}: {' '.join(str(x) for x in r)}\n")
        if float(cost).is_integer():
            f.write(f"Cost {int(round(cost))}\n")
        else:
            f.write(f"Cost {float(cost):.3f}\n")

# -------------------------------------------------
# 2. Main multi-run function
# -------------------------------------------------
from pathlib import Path
from collections import defaultdict
import time, csv, random, numpy as np

import time
import csv
from pathlib import Path

def _save_convergence_csv(path: str, history: list[tuple[int, float, int]]):
    """Save convergence history (eval, best_cost, routes) to CSV."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["eval", "best_cost", "routes"])
        for e, c, r in history:
            w.writerow([e, c, r])

def run_runs(
    vrp_path: str,
    algos: list[str],
    budget: int = 1000,
    runs: int = 3,
    seed_solutions: str | None = None,
    baseline_sol_path: str | None = None,
    output_dir: str = "results",
):
    """
    Run multiple algorithms on the same instance.
    Saves both a summary CSV and convergence CSVs for each run.

    Args:
        vrp_path: path to CVRPLIB instance (.vrp)
        algos: list of Nevergrad algorithm names (e.g. ["NGOpt", "DE"])
        budget: number of evaluations per run
        runs: number of independent repetitions
        seed_solutions: path to .sol file (optional, for hybrid runs)
        baseline_sol_path: optional baseline for comparison in summary table
        output_dir: folder for CSV outputs
    """
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    data = extract_cvrp_data(read(vrp_path))
    # inside run_runs, right after:
    # data = extract_cvrp_data(read(vrp_path))
    pdata = read(vrp_path)
    data = extract_cvrp_data(pdata)

    # 🔧 enforce VRPLIB CAPACITY from header (critical for X-n219-k73)
    cap_hdr = read_vrplib_capacity(vrp_path)
    if cap_hdr is not None and cap_hdr != data.capacity:
        print(f"[warn] PyVRP reported capacity {data.capacity}, but VRPLIB header says {cap_hdr}. Using header value.")
        data.capacity = int(cap_hdr)
    # Optional: baseline cost from .sol (for relative comparison)
    base_cost, base_routes = None, None
    

    rows = []
    instance_stem = Path(vrp_path).stem

    for algo in algos:
        for mode in ["unseeded", "seeded"] if seed_solutions else ["unseeded"]:
            for r in range(1, runs + 1):
                tag = f"{instance_stem}__{algo}__{mode}__run{r}"
                print(f"{algo:10s} {mode:10s} | run {r}/{runs}")

                start_t = time.time()
                best_cost, best_routes, hist = run_nevergrad(
                    data,
                    budget,
                    log_every=max(1, budget // 10),
                    algo_name=algo,
                    initial_sol_paths=seed_solutions if mode == "seeded" else None,
                )
                runtime = time.time() - start_t

                print(f"→ cost={best_cost:.3f} routes={len(best_routes)} runtime={runtime:.2f}s")

                # Save convergence history
                conv_path = Path(output_dir) / f"{tag}_convergence.csv"
                _save_convergence_csv(conv_path, hist)

                # Append row to results
                gap = None
                if base_cost and base_cost > 0:
                    gap = 100.0 * (best_cost - base_cost) / base_cost

                rows.append({
                    "instance": instance_stem,
                    "algo": algo,
                    "mode": mode,
                    "run": r,
                    "cost": best_cost,
                    "routes": len(best_routes),
                    "runtime_sec": runtime,
                    "gap_vs_baseline_%": gap,
                    "baseline_cost": base_cost,
                })

    # Write global summary CSV
    csv_path = Path(output_dir) / f"{instance_stem}_summary.csv"
    with open(csv_path, "w", newline="") as f:
        fieldnames = ["instance", "algo", "mode", "run", "cost", "routes", "runtime_sec", "gap_vs_baseline_%", "baseline_cost"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print(f"\n✅ Saved summary to {csv_path}")
    print(f"   (per-run convergence logs in {output_dir}/ )")

    return rows
    # --- Write CSV summary ---
    # --- Write CSV summary (with consistent columns) ---
    if rows:
        # Define a stable set of columns (include seed_files even if empty)
        fieldnames = [
            "instance", "algo", "mode", "run_id", "seed", "budget",
            "cost", "routes", "gap_percent_vs_perm_split",
            "runtime_sec", "sol_path", "seed_files",
        ]
        # Ensure every row has all fields
        for row in rows:
            for k in fieldnames:
                row.setdefault(k, "")  # empty for unseeded rows' seed_files

        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n✅ Saved run summary to {csv_path}")

    return rows

from collections import defaultdict
from pathlib import Path
import numpy as np

def summarize_runs_csv_perm_split_only(
    rows: list[dict],
    vrp_path: str,
    sol_path: str | None = None,   # if None -> sibling .sol next to the .vrp
):
    """
    Summarize multiple runs (rows from run_runs). Prints a compact table and
    compares ONLY against the perm+Split baseline (.sol flattened + Split),
    i.e., the exact same evaluation Nevergrad uses.
    """
    # Resolve .sol path (same dir/basename as instance if not provided)
    vrp_p = Path(vrp_path)
    if sol_path is None:
        sol_guess = vrp_p.with_suffix(".sol")
        sol_path = str(sol_guess) if sol_guess.exists() else None

    # Baseline: .sol as permutation + Split (Nevergrad-equivalent)
    baseline_cost = None
    baseline_routes = None
    if sol_path is not None:
        baseline_cost, baseline_routes, _ = sol_perm_split_cost(vrp_path, sol_path)

    groups = defaultdict(list)
    for r in rows:
        groups[(r["algo"], r["mode"])].append(r)

    print("\n=== AGGREGATE SUMMARY (perm+Split baseline ONLY) ===")
    if baseline_cost is not None:
        print(f"Baseline (perm+Split): cost={baseline_cost:.3f}  routes={baseline_routes}")

    header = (
        f"{'Algo':12} {'Mode':9} "
        f"{'Mean Cost':>12} {'Best Cost':>12} "
        f"{'Mean Routes':>12} {'Mean Rt(s)':>11} {'#Runs':>6}"
    )
    if baseline_cost is not None:
        header += f" {'Gap% vs perm+Split (mean)':>26} {'(best)':>8}"
    print(header)
    print("-" * len(header))

    out = {}
    for (algo, mode), lst in sorted(groups.items()):
        mean_cost   = float(np.mean([x["cost"] for x in lst]))
        best_cost   = float(np.min([x["cost"] for x in lst]))
        mean_routes = float(np.mean([x["routes"] for x in lst]))
        mean_rt     = float(np.mean([x["runtime_sec"] for x in lst]))
        n_runs      = len(lst)

        line = f"{algo:12} {mode:9} {mean_cost:12.3f} {best_cost:12.3f} {mean_routes:12.2f} {mean_rt:11.2f} {n_runs:6d}"

        stats = {
            "mean_cost": mean_cost,
            "best_cost": best_cost,
            "mean_routes": mean_routes,
            "mean_runtime_sec": mean_rt,
            "runs": n_runs,
        }

        if baseline_cost is not None:
            gap_mean = 100.0 * (mean_cost - baseline_cost) / baseline_cost
            gap_best = 100.0 * (best_cost - baseline_cost) / baseline_cost
            line += f" {gap_mean:26.3f} {gap_best:8.3f}"
            stats["gap_perm_split_mean_%"] = gap_mean
            stats["gap_perm_split_best_%"] = gap_best

        print(line)
        out[(algo, mode)] = stats

    return {
        "baseline_perm_split": {"cost": baseline_cost, "routes": baseline_routes},
        "by_group": out,
    }


from pathlib import Path
import pandas as pd
from datetime import datetime

# ==== CONFIG ====
ROOT = Path("CVRP_TEST")                  # folder with *.vrp / *.sol files
SEED_FOLDER = Path("SeededSolutions")      # folder with seed solutions

ALGOS   = ["NGOpt", "OnePlusOne", "DE"]
BUDGET  = 1000
RUNS    = 10
BKS_CSV = ROOT / "bks.csv"                # optional CSV with columns: instance,bks

# ==== OPTIONAL: load BKS from CSV ====
def load_bks_map(bks_csv: Path):
    if bks_csv.exists():
        df = pd.read_csv(bks_csv)
        name_col = "instance" if "instance" in df.columns else ("problem" if "problem" in df.columns else None)
        if name_col is None or "bks" not in df.columns:
            print(f"⚠️ {bks_csv} missing required columns ('instance'/'problem' + 'bks'). Ignoring.")
            return {}
        return {str(row[name_col]).strip(): int(row["bks"]) for _, row in df.iterrows()}
    return {}

BKS_MAP = load_bks_map(BKS_CSV)

# ==== Pretty printing helpers ====
def fmt_int(x):
    return "—" if x is None else f"{int(x):,}"

def extract_best_from_summary(summary: dict):
    for k in ["best_cost", "best", "best_obj", "best_value", "best_cost_overall"]:
        if k in summary:
            return int(summary[k])
    try:
        vals = [int(v) for k,v in summary.items() if "cost" in k.lower()]
        return min(vals) if vals else None
    except Exception:
        return None

# ==== Optional cost computation fallback ====
def try_compute_cost(vrp_path: Path, sol_path: Path):
    # Stub for evaluating a solution
    return None

# ==== Main batch (auto-discover X-*.vrp) ====
def run_for_all(chosen_initializer: str = "CHRISTOFIDES", save_filename="batch_summary_CHRISTOFIDES.csv"):
    """
    Batch run over all .vrp files in ROOT.
    chosen_initializer: "CHRISTOFIDES" or "PATH_CHEAPEST_ARC"
    """
    all_rows = []
    all_summaries = []

    vrps = sorted(ROOT.glob("X-*.vrp"))
    if not vrps:
        print("No X-*.vrp files found in", ROOT)
        return [], []

    print(f"\n== CVRP batch run started {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ==\n")
    print("Instance                BKS       Baseline     Seed       → Run Best")
    print("-"*74)

    # Map initializer to seed prefix
    seed_prefix_map = {
        "CHRISTOFIDES": "Christofides",
        "PATH_CHEAPEST_ARC": "PATH_CHEAPEST"
    }
    prefix = seed_prefix_map.get(chosen_initializer.upper(), chosen_initializer.upper())

    for vrp_file in vrps:
        stem = vrp_file.stem
        vrp  = vrp_file

        # Baseline solution (.sol) in ROOT
        base = ROOT / f"{stem}.sol"

        # Seed solution in SEED_FOLDER, chosen based on initializer
        seed = SEED_FOLDER / f"{prefix}_{stem}.sol"

        if not base.exists():
            print(f"{stem:<22} ⚠️ Missing baseline {base.name}. Skipping.")
            continue
        if not seed.exists():
            print(f"{stem:<22} ⚠️ Missing seed {seed.name}. Skipping.")
            continue

        # Get BKS: CSV → fallback → else unknown
        bks = BKS_MAP.get(stem)
        if bks is None:
            bks = try_compute_cost(vrp, base)

        baseline_cost = try_compute_cost(vrp, base)
        seed_cost     = try_compute_cost(vrp, seed)

        # Run the experiment using the chosen seed solution
        rows = run_runs(
            str(vrp), ALGOS, budget=BUDGET, runs=RUNS,
            seed_solutions=str(seed),
            baseline_sol_path=str(base)
        )

        summary = summarize_runs_csv_perm_split_only(rows, vrp_path=str(vrp), sol_path=None)
        best_cost = extract_best_from_summary(summary)

        print(f"{stem:<22} {fmt_int(bks):>8}   {fmt_int(baseline_cost):>10}   {fmt_int(seed_cost):>10}   → {fmt_int(best_cost):>9}")

        all_rows.extend(rows)
        all_summaries.append({"instance": stem, **summary})

    print("-"*74)
    print("Done.\n")

    try:
        pd.DataFrame(all_summaries).to_csv(save_filename, index=False)
        print(f"Saved: {save_filename}")
    except Exception as e:
        print(f"Note: could not save CSV ({e}).")

    return all_rows, all_summaries
