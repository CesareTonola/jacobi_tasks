import time
import numpy as np
import pinocchio as pin
from pathlib import Path
from meshcat_shapes import frame
from pinocchio.visualize import MeshcatVisualizer
from scipy.spatial.transform import Rotation as R
from ik_solver_py import IkSolver, IkSolverOptions

def display_target_frame(viewer, name, pose, length=0.2, radius=0.01):

    # Create the reference frame geometry and apply transformation
    frame(viewer[name], axis_length=length, axis_thickness=radius)
    viewer[name].set_transform(pose.homogeneous)

def main():
    
    # Load model + visual geometry
    model_path = Path(__file__).resolve().parent.parent.parent.parent / "robot-urdfs"
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
        return 1

    viz.loadViewerModel()
    viz.displayVisuals(True)

    # Spawn the robot ad neutral position
    viz.display(np.zeros(model.nq))

    # IK solver 
    frame_name = "abb_irb6700_flange"
    solver = IkSolver(urdf_filename)
    solver.verbose_ = False

    data = pin.Data(model)
    options = IkSolverOptions()

    while True:
        # Generate random target pose
        M = solver.get_random_pose_target(frame_name)
        pose_target = pin.SE3(M)

        # Show the target pose 
        display_target_frame(viz.viewer, "target", pose_target)
        input("\nPress Enter to compute IK for this target pose\n")

        success = False
        max_iter = 20
        for iter in range(max_iter):
            start = time.time()
            success, ik = solver.compute_ik(frame_name, M, options)
            duration = time.time() - start

            if success:
                print(f"IK found in {iter + 1} trials")
                print("IK:", ik.round(3))
                print(f"Computed in {duration * 1000:.4f} ms")

                # Error analysis
                pin.forwardKinematics(model, data, ik)
                pin.updateFramePlacements(model, data)
                frame_id = model.getFrameId(frame_name)
                final_pose = data.oMf[frame_id]
                error = pin.log6(final_pose.inverse() * pose_target).vector
                print("Final 6D error:", error.round(6))

                # Show final pose in MeshCat
                viz.display(ik)
                display_target_frame(viz.viewer, "ik_result", final_pose, 0.1, 0.005)

                break

        if not success:
            print("IK not found.")

        input("\nPress Enter to get another target pose...\n")
        print("-------------------------------------------")

if __name__ == "__main__":
    main()
