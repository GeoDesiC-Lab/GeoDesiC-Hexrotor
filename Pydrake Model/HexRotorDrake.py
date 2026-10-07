##### CONFIG
# must run the following in terminal to establish pydrake environment variables:
'''
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
'''
# will work out an automatica way to establish this later

##### PYDRAKE IMPORTS
# from pydrake.all import 

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, Propeller, PropellerInfo, CoulombFriction, BaseBodyJointType
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import AddFrameTriadIllustration
from pydrake.geometry import StartMeshcat, MeshcatVisualizer, MeshcatVisualizerParams, HalfSpace, ProximityProperties, AddContactMaterial

##### OTHER IMPORTS
from pathlib import Path
import argparse
import numpy as np

##### SELF-DEFINED IMPORTS
from utils.xacro import XacroToURDF
from utils.linear import LinearSys, LinearSysOut

##### USER INPUTS
simulation_duration = 5.0 # (s)
rotor_initial_position = [0.0, 0.0, 0.5] # (m)
rotor_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)

##### PARSING ARGUMENTS
argparser = argparse.ArgumentParser(description='HexRotor Simulation Model') # creating parser

argparser.add_argument('--frames', type=int, required=False, default=1, help='(1) for model frame visibility, (!=1) for no frames')
argparser.add_argument('--quats', type=int, required=False, default=1, help='(1) for quaternion base coordinates, (!=1) for rpy.')
argparser.add_argument('--linear', type=int, required=False, default=1, help='(1) for linear representation, (!=1) for nonlinear.')

args = argparser.parse_args() # getting the arguments

##### MAKING DIRECTORIES
MOD_DIR = Path('models')
UTI_DIR = Path('utils')

MOD_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
UTI_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist

##### DRAKE MODEL
builder = DiagramBuilder() # initiating the builder

##### MATHEMATICAL MODEL
print(f'\n=====CREATING PLANT=====')
plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=0.0) # creating the plant and scene graph

parser = Parser(plant) # initialize parser
inspector = scene_graph.model_inspector() # initialize inspector

# extracting constants
gravity = plant.gravity_field().gravity_vector()
g_mag = np.linalg.norm(gravity)

# adding floor to world
X_WG = RigidTransform.Identity() # identity transform

proximity_properties = ProximityProperties()
surface_friction = CoulombFriction(static_friction=0.7, dynamic_friction=0.5) # floor friction
AddContactMaterial(friction=surface_friction, properties=proximity_properties)

plant.RegisterCollisionGeometry(
    body=plant.world_body(), X_BG=X_WG, shape=HalfSpace(), name='ground_collision', properties=proximity_properties
)

# loading all models to parser
HexRotor = XacroToURDF(str(MOD_DIR / 'HexRotor.urdf.xacro'))

parser.AddModelsFromString(HexRotor, 'urdf')

##### CREATING PROPELLERS
# finding prop bodies and frames
rotor_prop_names = ['rotor_prop_1r', 'rotor_prop_1l', 'rotor_prop_2r', 'rotor_prop_2l','rotor_prop_3r', 'rotor_prop_3l'] # link names
rotor_prop_bodies = [plant.GetBodyByName(rotor_prop_name) for rotor_prop_name in rotor_prop_names] # model bodies

# adding the propellers
rotor_prop_info = []
for i, rotor_prop_body in enumerate(rotor_prop_bodies):
    thrust_ratio = 1.0 # thrust ratio for prop
    moment_ratio = 0.0 * (-1)**i # moment ratio for prop (pos and negative to signify direction of prop spin)

    X_RP = RigidTransform.Identity() # identity transform

    # can add some check to determine correct spin directions for each prop
    rotor_prop_info.append(PropellerInfo(rotor_prop_body.index(), X_BP=X_RP, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # propeller for split


##### ADDING VISUAL FRAMES
if args.frames: # if adding visual frames
    print(f'\n=====ADDING FRAMES=====')
    for i in range(plant.num_model_instances()): # for each model
        model_instance = ModelInstanceIndex(i) # get the model
        body_indices = plant.GetBodyIndices(model_instance) # get each body

        for body_index in body_indices: # for each body
            body = plant.get_body(body_index) 

            # add frame
            AddFrameTriadIllustration(
                scene_graph=scene_graph, plant=plant, body=body, length=0.15, radius=0.005
            )

##### CONFIGURING QUATERNIONS
if args.quats != 1: plant.SetBaseBodyJointType(BaseBodyJointType.kRpyFloatingJoint) # if not using quaternions

##### BUILDING MODEL
print(f'\n=====BUILDING MODEL=====')
plant.Finalize() # finalize the plant

# wiring diagram
propellers = builder.AddSystem(Propeller(rotor_prop_info)) # creating the propellers

# builder.Connect(plant.get_geometry_pose_output_port(), scene_graph.get_source_pose_port(plant.get_source_id()))
builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())
builder.Connect(propellers.get_spatial_forces_output_port(), plant.get_applied_spatial_force_input_port())

builder.ExportInput(propellers.get_command_input_port(), 'propeller_thrusts')
builder.ExportOutput(plant.get_state_output_port(), 'rotor_state')
builder.ExportOutput(scene_graph.get_query_output_port(), 'geometry_query')

##### VISUALIZING MODEL
print(f'\n=====STARTING VISUALIZATION=====')
meshcat = StartMeshcat() # initialize meshcat
# print(f'Open this URL in your browser: {meshcat.web_url()}')

visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, MeshcatVisualizerParams())

##### COMPLETING BUILD
diagram = builder.Build() # building final diagram

##### PLACING MODELS IN SCENE
context = diagram.CreateDefaultContext() # creating numerical context
plant_context = plant.GetMyMutableContextFromRoot(context)

rotor_instance = plant.GetModelInstanceByName('hex_rotor') # rotor instance

# placing the rotor
rotor_body = plant.GetBodyByName('rotor_base')
X_WR = RigidTransform(rotor_initial_rpy, rotor_initial_position) # target transform
plant.SetFreeBodyPose(context=plant_context, body=rotor_body, X_JpJc=X_WR)

##### INITIAL CONDITIONS
q_num = plant.num_positions() # number of q coords
v_num = plant.num_velocities() # number of velocities
u_num = propellers.get_command_input_port().size() # number of control inputs

v_zero = np.zeros(v_num) # setting velocities

# printing coordinate names for reference
print(f'\n=====GENERALIZED COORDINATES=====')
q_names = plant.GetPositionNames()
v_names = plant.GetVelocityNames()
for i, name in enumerate(q_names): print(f'q[{i}] -> {name}')
print('')
for i, name in enumerate(v_names): print(f'v[{i}] -> {name}')

# defining initial propeller thrusts
rotor_mass = plant.CalcTotalMass(plant_context, [rotor_instance])
prop_thrust = 1.00 * g_mag * rotor_mass / u_num # splitting thurst over all props
u_zero = prop_thrust * np.ones(u_num) # setting control

# u_zero[0] = 0.999 * u_zero[0]
# u_zero[2] = 0.999 * u_zero[2]
# u_zero[4] = 0.999 * u_zero[4]

# inputting initial conditions
plant.SetVelocities(plant_context, v_zero)

diagram_input_port = diagram.get_input_port(0)
diagram_input_port.FixValue(context, u_zero)

# linearizing system
if args.linear == 1: linear_system = LinearSys(system=diagram, context=context)

##### RUNNING SIMULATION
print(f'\n=====BEGINNING SIMULATION=====')
meshcat.StartRecording(set_visualizations_while_recording=True) # begin recording

simulator = Simulator(system=diagram, context=context)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)

simulator.AdvanceTo(simulation_duration) # advance simulation

# ending simulation
meshcat.StopRecording()
meshcat.PublishRecording() # watch simulation

##### OUTPUTTING DATA
print(f'\n=====SAVING RESULTS=====')
# linearized system
if args.linear == 1: LinearSysOut(linear_system=linear_system) # print linear system results

input(f'\nKeeping Meshcat alive!')