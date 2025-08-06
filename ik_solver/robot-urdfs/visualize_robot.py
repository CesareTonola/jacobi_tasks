import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from pathlib import Path
import sys

# Load model
model_path = Path(".").resolve()
urdf_filename = str(model_path / "jacobi_robot.urdf")
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_filename, package_dirs=[str(model_path)])
viz = MeshcatVisualizer(model, collision_model, visual_model)

try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz.loadViewerModel()


# Set configuration (default = home pose)
q0 = pin.neutral(model)
viz.display(q0)
viz.displayVisuals(True)

input("Press Enter to terminate...")
