#include <iostream>

#include "optimizer/trajectory_optimizer.h"
#include "optimizer/trajectory_sqp.h"
#include "optimizer/warm_start.h"
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Eigen::VectorXd;
using idto::optimizer::ProblemDefinition;
using idto::optimizer::SolverParameters;
using idto::optimizer::TrajectoryOptimizer;
using idto::optimizer::TrajectorySQP;
using idto::optimizer::TrajectoryOptimizerSolution;
using idto::optimizer::TrajectoryOptimizerStats;
using idto::optimizer::TrajectoryOptimizerState;
using idto::optimizer::WarmStart;
using idto::optimizer::ConvergenceReason;

void bind_trajectory_optimizer(py::module_& m) {
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.systems.framework");

  py::class_<TrajectoryOptimizer<double>>(m, "TrajectoryOptimizer")
      .def(py::init<const Diagram<double>*, const MultibodyPlant<double>*,
                    const ProblemDefinition&, const SolverParameters&>())
      .def("time_step", &TrajectoryOptimizer<double>::time_step)
      .def("num_steps", &TrajectoryOptimizer<double>::num_steps)
      .def("Solve",
           [](TrajectoryOptimizer<double>& optimizer,
              const std::vector<VectorXd>& q_guess,
              TrajectoryOptimizerSolution<double>* solution,
              TrajectoryOptimizerStats<double>* stats) {
             optimizer.Solve(q_guess, solution, stats);
           })
      .def("SolveFromWarmStart",
           [](TrajectoryOptimizer<double>& optimizer,
              WarmStart* warm_start,
              TrajectoryOptimizerSolution<double>* solution,
              TrajectoryOptimizerStats<double>* stats) {
             optimizer.SolveFromWarmStart(warm_start, solution, stats);
           })
      .def("CreateWarmStart", &TrajectoryOptimizer<double>::CreateWarmStart)
      .def("ResetInitialConditions",
           &TrajectoryOptimizer<double>::ResetInitialConditions)
      .def("UpdateNominalTrajectory",
           &TrajectoryOptimizer<double>::UpdateNominalTrajectory)
      .def("CreateState", 
          [](TrajectoryOptimizer<double>& self) {
               return std::make_unique<TrajectoryOptimizerState<double>>(self.num_steps(), self.diagram(), self.plant(),
                                       self.num_equality_constraints());
          })
      .def("EvalTau", &TrajectoryOptimizer<double>::EvalTau)
      .def("EvalTauJacobian", &TrajectoryOptimizer<double>::EvalTauJacobian)
      .def("EvalEqualityConstraintViolations", &TrajectoryOptimizer<double>::EvalEqualityConstraintViolations)
      .def("EvalEqualityConstraintJacobian", &TrajectoryOptimizer<double>::EvalEqualityConstraintJacobian)
      .def("params", &TrajectoryOptimizer<double>::params)
      .def("prob", &TrajectoryOptimizer<double>::prob);
  py::class_<TrajectorySQP<double>>(m, "TrajectorySQP")
      .def(py::init<const Diagram<double>*, const MultibodyPlant<double>*,
                    const ProblemDefinition&, const SolverParameters&>())
      .def("time_step", &TrajectorySQP<double>::time_step)
      .def("num_steps", &TrajectorySQP<double>::num_steps)
      .def("Solve",
           [](TrajectorySQP<double>& optimizer,
              const std::vector<VectorXd>& q_guess,
              TrajectoryOptimizerSolution<double>* solution,
              TrajectoryOptimizerStats<double>* stats) {
             optimizer.Solve(q_guess, solution, stats);
           })
      .def("SolveFromWarmStart",
           [](TrajectorySQP<double>& optimizer,
              WarmStart* warm_start,
              TrajectoryOptimizerSolution<double>* solution,
              TrajectoryOptimizerStats<double>* stats) {
             optimizer.SolveFromWarmStart(warm_start, solution, stats);
           })
      .def("CreateWarmStart", &TrajectorySQP<double>::CreateWarmStart)
      .def("ResetInitialConditions",
           &TrajectorySQP<double>::ResetInitialConditions)
      .def("UpdateNominalTrajectory",
           &TrajectorySQP<double>::UpdateNominalTrajectory)
      .def("CreateState", 
          [](TrajectorySQP<double>& self) {
               return std::make_unique<TrajectoryOptimizerState<double>>(self.num_steps(), self.diagram(), self.plant(),
                                       self.num_equality_constraints());
          })
      .def("EvalTau", &TrajectorySQP<double>::EvalTau)
      .def("EvalTauJacobian", &TrajectorySQP<double>::EvalTauJacobian)
      .def("EvalEqualityConstraintViolations", &TrajectorySQP<double>::EvalEqualityConstraintViolations)
      .def("EvalEqualityConstraintJacobian", &TrajectorySQP<double>::EvalEqualityConstraintJacobian)
      .def("params", &TrajectorySQP<double>::params)
      .def("prob", &TrajectorySQP<double>::prob);
  py::class_<WarmStart>(m, "WarmStart")
      // Warm start is not default constructible: it should be created
      // in python using the TrajectoryOptimizer.CreateWarmStart method.
      .def("set_q", &WarmStart::set_q)
      .def("get_q", &WarmStart::get_q)
      .def_readonly("Delta", &WarmStart::Delta)
      .def_readonly("dq", &WarmStart::dq)
      .def_readonly("dqH", &WarmStart::dqH);
  py::class_<TrajectoryOptimizerState<double>>(m, "TrajectoryOptimizerState")
      .def("q", &TrajectoryOptimizerState<double>::q)
    //   .def("mutable_q", &TrajectoryOptimizerState<double>::mutable_q)
      .def("set_q", &TrajectoryOptimizerState<double>::set_q)
      .def("v", &TrajectoryOptimizerState<double>::v)
      .def("a", &TrajectoryOptimizerState<double>::a)
      .def("tau", &TrajectoryOptimizerState<double>::tau);
      // Trajectory optimizer state is not default constructible: it should be created
      // in python using the TrajectoryOptimizer.CreateState method.
}
