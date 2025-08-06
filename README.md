# Jacobi Take-Home Tasks

This repository contains the solutions to the two take-home assignments:

1. [IK Solver](https://github.com/CesareTonola/jacobi_tasks/tree/main/ik_solver)
2. [Box Pose Estimation](https://github.com/CesareTonola/jacobi_tasks/tree/main/box_pose_estimation)

Each subproject includes a detailed explanation of the problem and how it was solved, along with relevant images and videos for visual validation.

---

## Installation & Usage

> **Note:** The code was developed and tested on **Ubuntu 22.04**. Compatibility with other systems is not guaranteed.

You can run the project either by **pulling a prebuilt Docker image** from Docker Hub, or by building it yourself using the provided `Dockerfile`.

---

### Option 1: Use Prebuilt Docker Image

The easiest way to get started is to pull the image from Docker Hub:

```bash
docker pull cesaretonola/jacobi_tasks:latest
```

Then, to launch the container with GUI/X11 and MeshCat support:

```bash
xhost +local:root

sudo docker run -it \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env XDG_RUNTIME_DIR=/tmp/runtime-docker \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume /tmp/runtime-docker:/tmp/runtime-docker \
    -p 7000:7000 \
    cesaretonola/jacobi_tasks:latest
```

> You can access MeshCat in your browser at: [http://localhost:7000/static/](http://localhost:7000/static/)

---

### Option 2: Build the Image Locally

If you prefer or need to build the Docker image yourself:

```bash
git clone https://github.com/CesareTonola/jacobi_tasks
cd jacobi_tasks
docker build -t jacobi_tasks .
```

Then run it as shown above by replacing the image name with `jacobi_tasks`.

---

### Running Scripts Inside the Container

Once you're inside the container, run the scripts as follows:

#### IK Solver

```bash
cd /jacobi_ws/src/jacobi_tasks/ik_solver/ik_solver/scripts/ik_solver
python3 compute_ik.py
```

This will start the IK computation and launch the MeshCat visualizer. For full details, see the [IK Solver](https://github.com/CesareTonola/jacobi_tasks/tree/main/ik_solver) section.

#### Box Pose Estimation

```bash
cd /jacobi_ws/src/jacobi_tasks/box_pose_estimation/scripts
python3 box_pose_estimation.py
```

This will build the point cloud and detect the box. For full details, see the [Box Pose Estimation](https://github.com/CesareTonola/jacobi_tasks/tree/main/box_pose_estimation) section.


