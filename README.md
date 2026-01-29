# pixel-to-action

Vision-based reinforcement learning robotics in simulation (starting with PyBullet).

## Setups (macOS / Linux)

### 0. Prereqs
- Git
- Conda (Miniconda / Anaconda)
- (Recommended) Python 3.10 or 3.11 via conda (do **not** use system Python 3.12 for PyBullet)

### 1. Clone
```bash
git clone <YOUR_REPO_URL>
cd pixel-to-action
```

### 2. Create the conda environment
We use conda to install all compiled dependencies (PyBullet, Torch, OpenCV).
```bash
conda env create -f environment.yaml -n robot-vision-rl
conda activate robot-vision-rl
```

### 3. Install the package (editable mode)
This makes `robot_vision_rl` importable everywhere.
```bash
pip install -e .
```
### 4. Enable pre-commit
```bash
pre-commit install
```

### 5. Sanity check
Run this to confirm everything is wired correctly:
```bash
python -c "import pybullet as p; import robot_vision_rl; print('All good')"
```

### 6. Identity setting in this repo
```bash
git config user.name "YOUR_NAME"
git config user.email "YOUR_EMAIL"
```

## Notebooks setup (Jupyter + VS Code)
This repo supports notebooks for debugging and visualisation.
### 1. Install notebook packages in the conda env
Install Jupyter + kernel tooling (either conda-forge or pip is fine; conda-forge is usually smoother):
```bash
pip install jupyterlab notebook ipykernel
```

<!-- ### 2. Register the kernel for VS Code
This makes the environment show up as a selectable kernel in VS Code:
```bash
python -m ipykernel install --user --name robot-vision-rl --display-name "pixel-to-action"
``` -->

### 2. Set Jupyter server root directory (Jupyter config)
You can force Jupyter to treat the repo as the file root by setting `c.ServerApp.root_dir = "<path-to-your-repo>"`

Generate config if you don’t already have it:
```bash
jupyter server --generate-config
```

Then edit the config:
```bash
vim ~/.jupyter/jupyter_server_config.py
```

Add (or modify) this line:
```bash
c.ServerApp.root_dir = "/ABSOLUTE/PATH/TO/pixel-to-action"
```

### 3. Setups in VS Code
- In VS Code, install the Jupyter extension from the Extensions marketplace.

- Open Settings (`Cmd` + `,`).

- Search for Notebook File Root.

- Set Jupyter: Notebook File Root to `${workspaceFolder}`.
