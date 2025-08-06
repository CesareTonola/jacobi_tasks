# Jacobi Take-Home Tasks

This repository contains the solutions to the two take-home assignments:

1. [IK Solver](https://github.com/CesareTonola/jacobi_tasks/tree/main/ik_solver)
2. [Box Pose Estimation](https://github.com/CesareTonola/jacobi_tasks/tree/main/box_pose_estimation)

Each subproject includes a detailed explanation of the problem and how it was solved, along with relevant images and videos for visual validation.

---

## Installation & Usage

> **Note:** The code was developed and tested on **Ubuntu 22.04**. Compatibility with other systems is not guaranteed.

A Conda environment is provided to simplify Python dependency management.

### 1. Setup the Workspace

Create a workspace (e.g. `~/jacobi_ws`), then clone this repository inside the `src` folder:

```bash
mkdir -p ~/jacobi_ws/src
cd ~/jacobi_ws/src
git clone https://github.com/CesareTonola/jacobi_tasks
```

### 2. Setup the Python Environment

Create and activate the conda environment:

```bash
cd ~/jacobi_ws/src/jacobi_tasks
conda env create -f conda_env.yaml
conda activate jacobi_task_env
```

---

## Running the Tasks

### IK Solver

This task requires building the C++ package first. Before, that, you need to install [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html). Then:

```bash
cd ~/jacobi_ws
colcon build --symlink-install
source install/setup.bash
```

Run the Python script:

```bash
cd ~/jacobi_ws/src/jacobi_tasks/ik_solver/ik_solver/scripts/ik_solver
python3 compute_ik.py
```

For full details, see the [IK Solver](https://github.com/CesareTonola/jacobi_tasks/tree/main/ik_solver) section.

---

### Box Pose Estimation

No compilation is needed. Run the script directly:

```bash
cd ~/jacobi_ws/src/jacobi_tasks/box_pose_estimation/scripts
python3 box_pose_estimation.py
```

For full details, see the [Box Pose Estimation](https://github.com/CesareTonola/jacobi_tasks/tree/main/box_pose_estimation) section.


