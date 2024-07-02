import klampt
from klampt.math import so3
# from klampt import IKObjective,IKSolver
from klampt.model import ik
import xml.etree.ElementTree as ET
import klampt.model
import numpy as np
from klampt.plan import robotplanning,cspace,robotcspace
from klampt.model.create import primitives
from klampt.model.geometry import Geometry3D
from klampt.model import collide
from klampt.io import resource
from klampt.model.trajectory import RobotTrajectory

def read_joint_limits(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # robot_name = root.get('name')
    # print("Robot name:", robot_name)
    minmax_dict = {}
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get("type")
        limit = joint.find('limit')
        if limit is not None:
            upper_limit = float(limit.get('upper'))
            lower_limit = float(limit.get('lower'))
            minmax_dict[joint_name] = {"min": lower_limit, "max": upper_limit}
        else:
            minmax_dict[joint_name] = {"min": 0, "max": 0}
    return minmax_dict


if __name__ == "__main__":
    import time
    np.set_printoptions(precision=2,suppress=True)
    urdf_path = r"./fanuc_m20id35_support/urdf/fanuc_m20id35_klampt.urdf"
    
    world = klampt.WorldModel()
    robot = world.loadRobot(urdf_path)
    print(f"load urdf joint limits:{robot.getJointLimits()}")

    minmax_dict = read_joint_limits(urdf_path)
    # set world_joint
    min_values = [0]
    max_values = [0]

    #set limitation of joint 1~6
    for ii in range(1,7):
        joint_name = f"joint_{ii}"
        # min_value = minmax_dict[joint_name]["min"]
        # max_value = minmax_dict[joint_name]["max"]

        min_value = -np.pi
        max_value = np.pi
        # print(f"{joint_name}: lower limit = {min_value} rad, upper limit = {max_value} rad.")
        min_values.append(min_value)
        max_values.append(max_value)

    #set limitation of tcp joint
    min_values.append(0)
    max_values.append(0)
    #comment next line to have a feasible result
    # robot.setJointLimits(min_values,max_values)
    print(f"set joint limits to:{robot.getJointLimits()}")


    # build a cube as obstacle
    # obj = world.makeRigidObject("obstacle")
    # obstacle = primitives.box(1,0.5,1)
    # obj.geometry().set(obstacle)
    # obj.appearance().setColor(1.0,0.1,0.1,1.0)
    # obj.geometry().translate((1.2,0,0))


    space = robotplanning.make_space(world,robot,edgeCheckResolution=0.1)
    # space = robotcspace.RobotCSpace(robot,collide.WorldCollider(world))
    qstart = [0.0, 0.6399999999999998, 0.5399999999999999, -0.6200000000000001, 0.0, -1.5707963267948966, 0.0, 0.0]
    qgoal = [0.0, -0.5, 0.6000000000000001, -0.18000000000000002, 0.0, -1.5707963267948966, 0.0, 0.0]

    settings = {
        "type":"sbl",
        "bidirectional":1,
        "knn":100,
        "connectionThreshold": 1,
        "perturbationRadius": 0.1, 
        # "randomizeFrequency":5000, 
        # "shortcut":1,
        # "restart":1,
        # "gridResolution":0.1
        }

    t0 = time.time()
    print("Creating planner...")
    #Manual construction of planner
    planner = cspace.MotionPlan(space, **settings)
    planner.setEndpoints(qstart,qgoal)

    print("Planner creation time",time.time()-t0)
    t0 = time.time()
    print("Planning...")
    numIters = 0
    for round in range(20):
        planner.planMore(1000)
        numIters += 1
        if planner.getPath() is not None:
            break
    print("Planning time,",numIters,"iterations",time.time()-t0)

    path = planner.getPath()
    if path is not None:
        print("Got a path with",len(path),"milestones")
        path_array = np.asarray(path)
        path_min_values = path_array.min(axis=0)
        path_max_values = path_array.max(axis=0)
        for ii,(min_value,max_value) in enumerate(zip(min_values,max_values)):
            print(f"joint limit {ii} :from {min_value} ~ {max_value}; path point {ii}: from {path_min_values[ii]} ~ {path_max_values[ii]}  ")
    else:
        print("No feasible path was found")

    #provide some debugging information
    V,E = planner.getRoadmap()
    print(len(V),"feasible milestones sampled,",len(E),"edges connected")

    # print("CSpace stats:")
    # spacestats = space.getStats()
    # for k in sorted(spacestats.keys()):
    #     print(" ",k,":",spacestats[k])

    # print("Planner stats:")
    # planstats = planner.getStats()
    # for k in sorted(planstats.keys()):
    #     print(" ",k,":",planstats[k])

    # if path:


    #     #visualize path as a Trajectory resource
    #     traj = RobotTrajectory(robot,range(len(path)),path)
    #     # traj = RobotTrajectory(robot,range(len(V)),V)

    #     resource.edit("Planned trajectory",traj,world=world)

