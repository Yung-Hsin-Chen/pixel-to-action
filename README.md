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
pip install -e . --no-deps
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


### 2. Setups in VS Code
- In VS Code, install the Jupyter extension from the Extensions marketplace.

- Open Settings (`Cmd` + `,` on macOS; `Ctrl` + `,` on Windows/Linux)

- Search for Notebook File Root.

- Set Jupyter: Notebook File Root to `${workspaceFolder}`.
