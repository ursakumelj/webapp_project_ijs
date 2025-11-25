# # from nevergrad_code import run_suite_compare
# # import streamlit as st
# # from pathlib import Path

# # # ----------------------------
# # # Streamlit UI
# # # ----------------------------
# # st.set_page_config(page_title="Solver with Nevergrad")
# # st.title("CVRP Nevergrad Optimization")

# # # --- Configure test problems ---
# # BASE_DIR = Path(__file__).parent
# # TEST_PROBLEMS = {
# #     "X-n101-k25": BASE_DIR / "CVRP_TEST/X-n101-k25.vrp",
# #     "X-n219-k73": BASE_DIR / "CVRP_TEST/X-n219-k73.vrp",
# #     "X-n322-k28": BASE_DIR / "CVRP_TEST/X-n322-k28.vrp",
# # }

# # problem_name = st.selectbox("Select test problem:", list(TEST_PROBLEMS.keys()))
# # vrp_path = TEST_PROBLEMS[problem_name]
# # sol_path_guess = Path(vrp_path).with_suffix(".sol")
# # sol_path = str(sol_path_guess) if sol_path_guess.exists() else None

# # # --- Parameters ---
# # RUNS = st.number_input("Number of independent runs", value=3, min_value=1, step=1)
# # BUDGET = st.number_input("Budget (number of evaluations)", value=500, min_value=1, step=100)

# # # --- Run button ---
# # if st.button("Run Optimization"):

# #     if not Path(vrp_path).exists():
# #         st.error(f"VRP file not found: {vrp_path}")
# #     elif sol_path is None:
# #         st.warning(f"No .sol file found next to {vrp_path}. Will run unseeded only.")

# #     else:
# #         st.info(f"Using seed solution: {sol_path}")

# #     # List of algorithms to run
# #     ALGOS = ["NGOpt", "OnePlusOne", "DE"]

# #     # Run the suite for each algorithm, with/without seed
# #     summary = run_suite_compare(
# #         vrp_path=str(vrp_path),
# #         algos=ALGOS,
# #         budget=int(BUDGET),
# #         sol_path=sol_path,
# #         log_every=0,          # silence inner logs
# #         use_split_decoder=True
# #     )

# #     # Display summary
# #     st.subheader("Optimization Summary")
    
# #     ref = summary.get("reference")
# #     results = summary.get("results", [])

# #     if ref:
# #         st.write(f"Re-evaluated .sol: cost = {ref['cost_abs']:.3f}, routes = {ref['routes']}")

# #     # Show results table
# #     import pandas as pd
# #     df = pd.DataFrame(results)
# #     st.dataframe(df)
# from nevergrad_code import run_suite_compare
# import streamlit as st
# from pathlib import Path
# import pandas as pd

# # ----------------------------
# # Streamlit UI
# # ----------------------------
# st.set_page_config(page_title="Solver with Nevergrad")
# st.title("CVRP Nevergrad Optimization")

# # --- Configure test problems ---
# BASE_DIR = Path(__file__).parent
# TEST_DIR = BASE_DIR / "CVRP_TEST"

# TEST_PROBLEMS = {
#     "X-n101-k25": TEST_DIR / "X-n101-k25.vrp",
#     "X-n219-k73": TEST_DIR / "X-n219-k73.vrp",
#     "X-n322-k28": TEST_DIR / "X-n322-k28.vrp",
# }

# problem_name = st.selectbox("Select test problem:", list(TEST_PROBLEMS.keys()))
# vrp_path = TEST_PROBLEMS[problem_name]

# # ----------------------------
# # User selects initial method
# # ----------------------------
# init_method = st.radio(
#     "Choose initial solution generator:",
#     ["CHRISTOFIDES", "PATH_CHEAPEST_ARC"]
# )

# # Build seed filename based on choice
# seed_prefix = "Christofides" if init_method == "CHRISTOFIDES" else "PATH_CHEAPEST"
# seed_filename = f"{seed_prefix}_{problem_name}.sol"
# seed_path = TEST_DIR / seed_filename

# if seed_path.exists():
#     sol_path = str(seed_path)
#     st.success(f"Using seed file: {seed_filename}")
# else:
#     sol_path = None
#     st.warning(f"Seed file '{seed_filename}' not found — running without seed.")

# # ----------------------------
# # Parameters
# # ----------------------------
# RUNS = st.number_input("Number of independent runs", value=3, min_value=1, step=1)
# BUDGET = st.number_input("Budget (number of evaluations)", value=500, min_value=1, step=100)

# # ----------------------------
# # Run button
# # ----------------------------
# if st.button("Run Optimization"):

#     if not vrp_path.exists():
#         st.error(f"VRP file not found: {vrp_path}")
#     else:
#         st.info(f"Running Nevergrad optimization for problem: {problem_name}")

#         ALGOS = ["NGOpt", "OnePlusOne", "DE"]

#         summary = run_suite_compare(
#             vrp_path=str(vrp_path),
#             algos=ALGOS,
#             budget=int(BUDGET),
#             sol_path=sol_path,      
#             log_every=0,
#             use_split_decoder=True
#         )

#         # Display summary
#         st.subheader("Optimization Summary")

#         ref = summary.get("reference")
#         results = summary.get("results", [])

#         if ref:
#             st.write(f"Re-evaluated seed solution: cost = {ref['cost_abs']:.3f}, routes = {ref['routes']}")

#         df = pd.DataFrame(results)
#         st.dataframe(df)
import streamlit as st
from pathlib import Path
import pandas as pd
from nevergrad_code import run_runs

# ----------------------------
# Streamlit UI
# ----------------------------
st.set_page_config(page_title="Solver with Nevergrad", page_icon="🚚")
st.title("CVRP Nevergrad Optimization")

# --- Configure test problems ---
BASE_DIR = Path(__file__).parent
TEST_DIR = BASE_DIR / "CVRP_TEST"

TEST_PROBLEMS = {
    "X-n101-k25": TEST_DIR / "X-n101-k25.vrp",
    "X-n219-k73": TEST_DIR / "X-n219-k73.vrp",
    "X-n322-k28": TEST_DIR / "X-n322-k28.vrp",
}

problem_name = st.selectbox("Select test problem:", list(TEST_PROBLEMS.keys()))
vrp_path = TEST_PROBLEMS[problem_name]

# --- Choose initial solution ---
initializer = st.radio("Choose initial solution:", ["CHRISTOFIDES", "PATH_CHEAPEST_ARC"])
seed_prefix = "Christofides" if initializer == "CHRISTOFIDES" else "PATH_CHEAPEST"
seed_path = TEST_DIR / f"{seed_prefix}_{problem_name}.sol"

if seed_path.exists():
    st.success(f"Using seed solution: {seed_path.name}")
else:
    st.warning(f"Seed solution '{seed_path.name}' not found — running unseeded only.")
    seed_path = None

# --- Parameters ---
ALGOS = ["NGOpt", "OnePlusOne", "DE"]
RUNS = st.number_input("Number of independent runs", value=5, min_value=1, step=1)
BUDGET = st.number_input("Budget (number of evaluations)", value=500, min_value=1, step=100)

# --- Run button ---
if st.button("Run Optimization"):

    if not vrp_path.exists():
        st.error(f"VRP file not found: {vrp_path}")
    else:
        st.info(f"Running Nevergrad optimization for problem: {problem_name}")

        # Run the Nevergrad experiments
        rows = run_runs(
            str(vrp_path),
            algos=ALGOS,
            budget=BUDGET,
            runs=RUNS,
            seed_solutions=str(seed_path) if seed_path else None
        )

        # Convert to DataFrame
        df = pd.DataFrame(rows)

        # Compute baseline (per-run seeded solution cost)
        baseline_cost = df[df["mode"]=="seeded"]["cost"].min()

        # Aggregate summary
        agg_list = []
        for (algo, mode), group in df.groupby(["algo","mode"]):
            mean_cost = group["cost"].mean()
            best_cost = group["cost"].min()
            mean_routes = group["routes"].mean()
            mean_rt = group["runtime_sec"].mean()
            n_runs = len(group)
            gap = ((mean_cost - baseline_cost)/baseline_cost*100) if baseline_cost else None

            agg_list.append({
                "Algo": algo,
                "Mode": mode,
                "Mean Cost": round(mean_cost,1),
                "Best Cost": best_cost,
                "Mean Routes": round(mean_routes,2),
                "Mean Rt(s)": round(mean_rt,2),
                "#Runs": n_runs,
                "Gap% vs baseline": round(gap,2) if gap is not None else None
            })

        df_agg = pd.DataFrame(agg_list)
        st.subheader("Aggregate Summary")
        st.dataframe(df_agg)
