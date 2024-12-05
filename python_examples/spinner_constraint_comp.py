#!/usr/bin/env python

##
#
# An example of using the python bindings to solve the spinner optimization
# problem.
#
##

import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 16})

from pydrake.all import (
    StartMeshcat,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    AddDefaultVisualization,
    Parser,
    PiecewisePolynomial,
    Simulator,
    ConstantValueSource,
    Value,
    LogVectorOutput,
    JointActuatorIndex,
    PdControllerGains,
    DiscreteContactApproximation,
)
import time
import numpy as np
from mpc_utils import Interpolator, StoredTrajectory

from pyidto import (
    TrajectoryOptimizer,
    TrajectoryOptimizerSolution,
    TrajectoryOptimizerStats,
    SolverParameters,
    ProblemDefinition,
    FindIdtoResource,
)


def define_spinner_optimization_problem():
    """
    Create a problem definition for the spinner.
    """
    problem = ProblemDefinition()
    problem.num_steps = 40
    problem.q_init = np.array([0.3, 1.5, 0.0])
    problem.v_init = np.array([0.0, 0.0, 0.0])
    problem.Qq = 1.0 * np.eye(3)
    problem.Qv = 0.1 * np.eye(3)
    problem.R = np.diag([0.1, 0.1, 1e3])
    problem.Qf_q = 10 * np.eye(3)
    problem.Qf_v = 0.1 * np.eye(3)

    q_nom = []   # Can't use list comprehension here because of Eigen conversion
    v_nom = []
    for i in range(problem.num_steps + 1):
        q_nom.append(np.array([0.3, 1.5, 2.0]))
        v_nom.append(np.array([0.0, 0.0, 0.0]))
    problem.q_nom = q_nom
    problem.v_nom = v_nom

    return problem


def define_spinner_solver_parameters(iters):
    """
    Create a set of solver parameters for the spinner.
    """
    params = SolverParameters()

    params.max_iterations = iters
    params.scaling = True
    params.equality_constraints = True
    params.Delta0 = 1e1
    params.Delta_max = 1e5
    params.num_threads = 4

    params.contact_stiffness = 200
    params.dissipation_velocity = 0.1
    params.smoothing_factor = 0.01
    params.friction_coefficient = 0.5
    params.stiction_velocity = 0.05

    params.verbose = False

    return params


def define_spinner_initial_guess(num_steps):
    """
    Create an initial guess for the spinner
    """
    q_guess = []
    for i in range(num_steps + 1):
        q_guess.append(np.array([0.3, 1.5, 2.0 * i / num_steps]))

    return q_guess


def visualize_trajectory(q, time_step, model_file, meshcat=None):
    """
    Display the given trajectory (list of configurations) on meshcat
    """
    # Create a simple Drake diagram with a plant model
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    Parser(plant).AddModels(model_file)
    plant.Finalize()

    # Connect to the meshcat visualizer
    AddDefaultVisualization(builder, meshcat)

    # Build the system diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    plant.get_actuation_input_port().FixValue(plant_context,
                                              np.zeros(plant.num_actuators()))

    # Step through q, setting the plant positions at each step
    meshcat.StartRecording()
    for k in range(len(q)):
        diagram_context.SetTime(k * time_step)
        plant.SetPositions(plant_context, q[k])
        diagram.ForcedPublish(diagram_context)
        time.sleep(time_step)
    meshcat.StopRecording()
    meshcat.PublishRecording()


if __name__ == "__main__":
    # Start up meshcat (for viewing the result)
    meshcat = StartMeshcat()

    # Absolute path of the model file that we'll use
    model_file = FindIdtoResource("idto/models/spinner_friction.urdf")

    # Specify a cost function and target trajectory
    problem = define_spinner_optimization_problem()

    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    print("iters, const_viol, final_err")
    for i, iters in enumerate([30, 60, 500]):

        # Specify solver parameters, including contact modeling parameters
        params = define_spinner_solver_parameters(iters)

        # Specify an initial guess
        q_guess = define_spinner_initial_guess(problem.num_steps)

        # Create the optimizer object
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.05)
        Parser(plant).AddModels(model_file)
        plant.Finalize()
        diagram = builder.Build()
        opt = TrajectoryOptimizer(diagram, plant, problem, params)

        # Allocate some structs that will hold the solution
        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()

        # Solve the optimization problem
        opt.Solve(q_guess, solution, stats)

        # Store some results
        const_viol = stats.h_norms[-1]
        t_opt = np.linspace(0, opt.time_step() * opt.num_steps(), opt.num_steps() + 1)
        x_opt = np.concatenate((solution.q, solution.v), axis=1).T

        # Put the solution into a StoredTrajectory object
        time_steps = np.linspace(
            0, opt.time_step() * opt.num_steps(), opt.num_steps() + 1)
        q_knots = np.array(solution.q).T
        v_knots = np.array(solution.v).T
        tau_knots = solution.tau
        tau_knots.append(solution.tau[-1])  # Repeat the last control input
        tau_knots = np.array(tau_knots).T

        # Create the StoredTrajectory object
        trajectory = StoredTrajectory()
        trajectory.start_time = 0.0
        trajectory.q = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, q_knots)
        trajectory.v = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, v_knots)
        trajectory.tau = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, tau_knots)
        
        # Set up a simulation that will play roll out u_opt open-loop
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.05)
        # plant.set_discrete_contact_approximation(
        #     DiscreteContactApproximation.kSimilar)
        models = Parser(plant).AddModels(model_file)

        # Add implicit PD controllers (must use kLagged or kSimilar)
        # Kp = 100 * np.ones(plant.num_actuators())
        # Kd = 10 * np.ones(plant.num_actuators())
        # actuator_indices = [JointActuatorIndex(
        #     i) for i in range(plant.num_actuators())]
        # for actuator_index, Kp, Kd in zip(actuator_indices, Kp, Kd):
        #     plant.get_joint_actuator(actuator_index).set_controller_gains(
        #         PdControllerGains(p=Kp, d=Kd))

        plant.Finalize()

        Bq = np.array([[1, 0, 0], [0, 1, 0]])
        Bv = np.array([[1, 0, 0], [0, 1, 0]])
        interpolator = builder.AddSystem(Interpolator(Bq, Bv))
        stored_source = builder.AddSystem(
            ConstantValueSource(Value(trajectory))
        )
        builder.Connect(
            stored_source.get_output_port(),
            interpolator.GetInputPort("trajectory")
        )
        builder.Connect(
            interpolator.GetOutputPort("control"),
            plant.get_actuation_input_port()
        )
        # builder.Connect(
        #     interpolator.GetOutputPort("state"),
        #     plant.get_desired_state_input_port(models[0])
        # )

        logger = LogVectorOutput(plant.get_state_output_port(), builder, 0.05)
        AddDefaultVisualization(builder, meshcat)
        diagram = builder.Build()

        # Set initial conditions
        diagram_context = diagram.CreateDefaultContext()
        plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
        plant.SetPositions(plant_context, x_opt[0:3, 0])
        plant.SetVelocities(plant_context, x_opt[3:6, 0])

        # Run the simulation
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(-1.0)
        meshcat.StartRecording()
        simulator.AdvanceTo(2.0)
        meshcat.StopRecording()
        meshcat.PublishRecording()

        # Recover the trajectory from the logger
        log = logger.FindLog(diagram_context)
        t_sim = log.sample_times()
        x_sim = log.data()

        plt.subplot(3, 1, i+1)
        plt.title(f"$||h||$={const_viol:.2e}")
        plt.plot(t_sim, x_sim[2, :], label="Actual", linewidth=4, linestyle="-", color='k')
        plt.plot(t_opt, x_opt[2, :], label="Predicted", linewidth=4, linestyle=":", color='k')
        if i == 0:
            plt.legend()

        # Compute the error in the final state
        final_angle_diff = np.abs(x_sim[2, -1] - x_opt[2, -1])
        print(f"{iters}, {const_viol}, {final_angle_diff}")

    fig.supylabel("Spinner Angle (rad)")
    fig.supxlabel("Time (s)")
    plt.tight_layout() 
    plt.show()


