ENV_NAME="webpage_env"
PYTHON_VERSION="3.13"
DISPLAY_NAME="CVRP Env"

# Create environment
conda create -n $ENV_NAME python=$PYTHON_VERSION -y

# Install core packages
conda install -n $ENV_NAME -y jupyter ipykernel numpy pandas matplotlib seaborn re typing -c conda-forge

# Install pip packages
conda run -n $ENV_NAME pip install --upgrade pyvrp nevergrad ortools streamlit plotly matplotlib numpy pandas seaborn

# Register Jupyter kernel
conda run -n $ENV_NAME python -m ipykernel install --user --name $ENV_NAME --display-name "$DISPLAY_NAME"
