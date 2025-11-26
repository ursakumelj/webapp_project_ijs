import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import nevergrad as ng

# Benchmark functions
def sphere(x):
    return sum(xi**2 for xi in x)

def rastrigin(x):
    return 10*len(x) + sum(xi**2 - 10*np.cos(2*np.pi*xi) for xi in x)

def rosenbrock(x):
    return sum(100*(x[i+1] - x[i]**2)**2 + (x[i] - 1)**2 for i in range(len(x)-1))

# Optimization function with parameter tracking
def run_optimization(algorithm, benchmark_func, dimensions, budget):
    """
    Function parameters:
        - algorithm: name of the evolutionary alg. from nevergrad library
        - benchmark_func: test function being optimized (sphere, rastrigin, rosenbrock)
        - dimensions: the dimension of the space
        - budget: maximum number of function evaluations allowed
    
    Returns:
        - best value: the lowest function value found
        - best parameters at this best value
        - history of all values for convergence trace
        - parameters history
    """
    parametrization = ng.p.Array(shape=(dimensions,), lower=-5.2, upper=5.2)
    optimizer = ng.optimizers.registry[algorithm](parametrization=parametrization, budget=budget)
    
    history = []
    param_history = []
    best_so_far = float('inf')
    best_params = None
    
    for _ in range(budget):
        x = optimizer.ask()
        value = benchmark_func(x.value)
        optimizer.tell(x, value) # gives the optimizer feedback about how the candidate performed
        
        if value < best_so_far:
            best_so_far = value
            best_params = x.value.copy() # copy is for storying actual values and not references
        
        history.append(best_so_far)
        param_history.append(best_params.copy() if best_params is not None else np.zeros(dimensions))
    
    return {
        "best_value": best_so_far,
        "best_params": best_params,
        "history": history,
        "param_history": param_history
    }

# ===== WEB APP
st.set_page_config(page_title="Benchmarking Evolutionary Algorithms")
st.title("Evolutionary algorithm benchmarking tool")

st.info("If the page fails to load properly, try refreshing it.")

# ===== Sidebar Controls =====
with st.sidebar:
    st.header("⚙️ Configuration")
    function_name = st.selectbox(
        "Benchmark function",
        ["Sphere", "Rastrigin", "Rosenbrock"],
        index=0
    )
    algorithms = st.multiselect(
        "Algorithms to compare",
        ["DE", "PSO", "OnePlusOne", "GeneticDE"],
        default=["DE", "PSO"]
    )
    dimensions = st.slider("Dimensions", 1, 20, 3)
    budget = st.slider("Evaluation budget", 100, 5000, 1000)
    num_runs = st.slider("Number of runs", 1, 30, 5)
    run_button = st.button("🚀 Run benchmark")

# Expected optimal solutions
optima = {
    "Sphere": {
        "value": 0.0,
        "params": np.zeros(dimensions),
        "description": "All parameters (vector components) should be exactly 0"
    },
    "Rastrigin": {
        "value": 0.0,
        "params": np.zeros(dimensions),
        "description": "All parameters (vector components) should be exactly 0"
    },
    "Rosenbrock": {
        "value": 0.0,
        "params": np.ones(dimensions),
        "description": "All parameters (vector components) should be exactly 1"
    }
}

# ==== Explanation of algorithms ====
st.markdown("---") 
with st.expander("📚 Explanation of the algorithms"):
    st.subheader("Differential Evolution (DE)")
    st.write("""
        Differential Evolution (DE) is a population-based optimization algorithm that works by iteratively refining a set of candidate solutions. 
        Unlike gradient-based methods, DE doesn't require the problem to be differentiable. Instead, it generates new trial solutions by intelligently combining (mutating and crossing over) existing solutions within its population. 
        If a trial solution outperforms an existing one, it replaces it, allowing the population to progressively evolve towards better solutions over many generations, effectively navigating complex and often non-continuous search spaces.
    """)
    st.markdown("---") 
    st.subheader("Particle Swarm Optimization (PSO)")
    st.write("""
        Particle Swarm Optimization (PSO) is a computational method that optimizes a problem by iteratively trying to improve a candidate solution with regard to a given measure of quality.
        It solves a problem by having a population of candidate solutions, here dubbed particles, and moving these particles around in the search space according to simple mathematical formula over the particle's position and velocity.
        Each particle's movement is influenced by its local best known position, but is also guided toward the best known positions in the search space, which are updated as better positions are found by other particles.
    """)
    st.markdown("---")
    st.subheader("OnePlusOne Evolution Strategy (+1 ES)")
    st.write("""
        The (1+1) Evolution Strategy is one of the simplest forms of evolution strategies.
        It maintains a single parent solution and generates a single offspring by applying mutation.
        If the offspring is better than or equal to the parent, the offspring replaces the parent.
        It's often used for continuous optimization and focuses on self adaptation of mutation step sizes.
    """)
    st.markdown("---")
    st.subheader("Genetic Algorithm with Differential Evolution (GeneticDE)")
    st.write("""
        This likely refers to a hybrid algorithm combining elements of Genetic Algorithms (GA) and Differential Evolution (DE).
        Typically, a GA involves selection, crossover, and mutation operators, while DE uses vector differences for mutation.
        A hybrid approach might use DE's mutation strategy within a GA framework, or integrate GA's crossover with DE's population management, aiming to leverage the strengths of both paradigms.
    """)
st.markdown("---")

# ===== Main frame =====
if run_button and algorithms:
    functions = {
        "Sphere": sphere,
        "Rastrigin": rastrigin,
        "Rosenbrock": rosenbrock
    }
    benchmark_func = functions[function_name]
    expected = optima[function_name]
    
    # Run optimizations
    results = {}
    progress_bar = st.progress(0)
    status_text = st.empty()
    
    for i, algo in enumerate(algorithms):
        status_text.text(f"Running {algo} ({i+1}/{len(algorithms)})...")
        algo_results = []
        
        for run in range(num_runs):
            result = run_optimization(algo, benchmark_func, dimensions, budget)
            algo_results.append(result)
            progress_bar.progress((i * num_runs + run + 1) / (len(algorithms) * num_runs))
        
        results[algo] = algo_results
    
    status_text.text("Benchmark complete! ✅")
    progress_bar.empty()
    
    # ===== Results display =====
    st.header("📊 Results summary")
    
    # Optimal solution info
    with st.expander("🎯 Expected optimal solution", expanded=True):
        st.markdown(f"""
        **Function:** `{function_name}`  
        **Optimal value:** `{expected['value']}`  
        **Optimal parameters:** `{np.round(expected['params'], 4)}`  
        **Description:** {expected['description']}
        """)
    
    # Create results table
    summary_data = []
    for algo in algorithms:
        best_values = [r["best_value"] for r in results[algo]]
        best_params = [r["best_params"] for r in results[algo]]
        best_idx = np.argmin(best_values)
        
        if num_runs == 1:
            summary_data.append({
                "Algorithm": algo,
                "Best value": best_values[0],
                "Parameters found": np.round(best_params[0], 4),
            })
        else:
            summary_data.append({
                "Algorithm": algo,
                "Best value": np.min(best_values),
                "Mean": np.mean(best_values),
                "Median": np.median(best_values),
                "Std dev": np.std(best_values),
                "Best parameters": np.round(best_params[best_idx], 4),
            })
    
    df = pd.DataFrame(summary_data)
    
    # Format pandas DataFrame
    num_cols = [col for col in df.columns if col not in ["Algorithm", "Best parameters", "Parameters found"]]
    format_dict = {col: "{:.4e}" for col in num_cols}
    st.dataframe(df.style.format(format_dict))
    
    # ===== Convergence plots =====
    st.header("📈 Convergence analysis")
    
    # Main comparison plot
    fig = go.Figure()
    for algo in algorithms:
        avg_convergence = np.mean([r["history"] for r in results[algo]], axis=0)
        fig.add_trace(go.Scatter(
            x=list(range(len(avg_convergence))),
            y=avg_convergence,
            mode='lines',
            name=algo,
            hovertemplate=(
                f"<b>Algorithm</b>: {algo}<br>"
                "<b>Evaluation</b>: %{x}<br>"
                "<b>Best Value</b>: %{y:.4e}<extra></extra>"
            )
        ))
    
    fig.add_hline(
        y=expected['value'],
        line_dash="dot",
        line_color="red",
        annotation_text="Optimum",
        annotation_position="bottom right"
    )
    
    fig.update_layout(
        title=f"Average convergence ({function_name}, {dimensions}D)",
        xaxis_title="Evaluations",
        yaxis_title="Best-so-far value",
        yaxis_type="log",
        hovermode="x unified"
    )
    st.plotly_chart(fig, use_container_width=True)
    
    # Individual algorithm plots
    for algo in algorithms:
        with st.expander(f"🔍 {algo} Detailed results", expanded=False):
            col1, col2 = st.columns(2)
            
            with col1:
                st.subheader("📝 Run statistics")
                run_data = []
                for i, run in enumerate(results[algo]):
                    run_data.append({
                        "Run": i+1,
                        "Best value": run["best_value"],
                        "Parameters": np.round(run["best_params"], 4),
                    })
                st.dataframe(pd.DataFrame(run_data))
            
            with col2:
                st.subheader("📉 Convergence")
                fig = go.Figure()
                for i, run in enumerate(results[algo]):
                    fig.add_trace(go.Scatter(
                        x=list(range(len(run["history"]))),
                        y=run["history"],
                        mode='lines',
                        name=f"Run {i+1}",
                        opacity=0.7,
                        hovertemplate=(
                            f"<b>Run</b>: {i+1}<br>"
                            "<b>Evaluation</b>: %{x}<br>"
                            "<b>Value</b>: %{y:.4e}<br>"
                            f"<b>Params</b>: {np.round(run['best_params'], 4)}"
                            "<extra></extra>"
                        )
                    ))
                
                fig.update_layout(
                    title=f"{algo} Convergence",
                    yaxis_type="log",
                    showlegend=True
                )
                st.plotly_chart(fig, use_container_width=True)
                

elif run_button and not algorithms:
    st.warning("Please select at least one algorithm to compare!")
else:
    st.info("Configure your benchmark in the sidebar and click 'Run Benchmark'")

# ===== Footer =====
st.sidebar.markdown("""
**📚 Goal of this tool:**
- Compare evolutionary algorithms
- Visualize convergence
""")