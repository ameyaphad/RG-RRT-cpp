#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/ODESolver.h>
#include <ompl/config.h>
#include <Eigen/Core>  // For Eigen::VectorXd
#include <cmath>
#include <iostream>
#include <fstream> // For writing solution to file
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/PathGeometric.h>
#include "RG-RRT.h"


// Constants for pendulum system
constexpr double GRAVITY = 9.81; // Gravity constant
constexpr double MIN_ANG_VEL = -10.0; // Min angular velocity
constexpr double MAX_ANG_VEL = 10.0;  // Max angular velocity

namespace ob = ompl::base;
namespace oc = ompl::control;

// Define the ODE for the pendulum system
void pendulumODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    const double theta = q[0];
    const double omega = q[1];
    const double torque = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];

    qdot.resize(2);
    qdot[0] = omega; // theta_dot = omega
    qdot[1] = -GRAVITY * std::cos(theta) + torque; // omega_dot = -g * cos(theta) + torque
}

// Define the pendulum validity checker
bool isValidState(const ob::State *state)
{
    const auto *pendulumState = state->as<ob::RealVectorStateSpace::StateType>();

    double omega = pendulumState->values[1];

    // Check if angular velocity is within the valid range
    if (omega < MIN_ANG_VEL || omega > MAX_ANG_VEL)
        return false;

    return true; // State is valid
}

// Projection for the KPIECE planner
class PendulumProjection : public ob::ProjectionEvaluator
{
public:
    PendulumProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space) {}

    unsigned int getDimension() const override
    {
        return 2; // Project onto theta and omega
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *pendulumState = state->as<ob::RealVectorStateSpace::StateType>();
        projection[0] = pendulumState->values[0]; // theta
        projection[1] = pendulumState->values[1]; // omega
    }
};


// Function for planning with RRT
void planPendulumwithRRT(double torqueLimit, const std::string &filename)
{
    // Create a state space for the pendulum (2D: [theta, omega])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set bounds for the pendulum's state space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);  // Lower bound for theta
    bounds.setHigh(0, M_PI);  // Upper bound for theta
    bounds.setLow(1, MIN_ANG_VEL);  // Lower bound for angular velocity
    bounds.setHigh(1, MAX_ANG_VEL); // Upper bound for angular velocity
    stateSpace->setBounds(bounds);

    // Create a control space for the pendulum (1D: torque)
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 1);

    // Set bounds for the control space (torque)
    ob::RealVectorBounds controlBounds(1);
    controlBounds.setLow(-torqueLimit); // Min torque
    controlBounds.setHigh(torqueLimit); // Max torque
    controlSpace->setBounds(controlBounds);

    // Create a simple setup
    oc::SimpleSetup ss(controlSpace);

    // Set the state validity checker
    ss.setStateValidityChecker([](const ob::State *state) { return isValidState(state); });

    // Set the ODE solver and system
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE);
    ss.getSpaceInformation()->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    // Define the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = -M_PI / 2; // Initial theta
    start[1] = 0.0;       // Initial omega (at rest)

    ob::ScopedState<> goal(stateSpace);
    goal[0] = M_PI / 2; // Goal theta (pendulum pointing up)
    goal[1] = 0.0;      // Goal omega (at rest)

    // Set start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Set the planner to RRT
    auto planner = std::make_shared<oc::RRT>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Attempt to solve the planning problem
    ob::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        std::cout << "Found solution with RRT. Writing to file " << filename << std::endl;

        // Write the solution path to a file
        std::ofstream file(filename);
        if (file.is_open())
        {
            auto pathControl=ss.getSolutionPath();
            auto pathGeometric = pathControl.asGeometric();

            pathGeometric.printAsMatrix(file);  // Output solution path to a file
            file.close();
        }
        else
        {
            std::cerr << "Unable to open file for writing!" << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found." << std::endl;
    }
}

void planPendulumwithRGRRT(double torqueLimit, const std::string &filename)
{
    // Create a state space for the pendulum (2D: [theta, omega])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set bounds for the pendulum's state space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);  // Lower bound for theta
    bounds.setHigh(0, M_PI);  // Upper bound for theta
    bounds.setLow(1, MIN_ANG_VEL);  // Lower bound for angular velocity
    bounds.setHigh(1, MAX_ANG_VEL); // Upper bound for angular velocity
    stateSpace->setBounds(bounds);

    // Create a control space for the pendulum (1D: torque)
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 1);

    // Set bounds for the control space (torque)
    ob::RealVectorBounds controlBounds(1);
    controlBounds.setLow(-torqueLimit); // Min torque
    controlBounds.setHigh(torqueLimit); // Max torque
    controlSpace->setBounds(controlBounds);

    // Create a simple setup
    oc::SimpleSetup ss(controlSpace);

    // Set the state validity checker
    ss.setStateValidityChecker([](const ob::State *state) { return isValidState(state); });

    // Set the ODE solver and system
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE);
    ss.getSpaceInformation()->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    // Define the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = -M_PI / 2; // Initial theta
    start[1] = 0.0;       // Initial omega (at rest)

    ob::ScopedState<> goal(stateSpace);
    goal[0] = M_PI / 2; // Goal theta (pendulum pointing up)
    goal[1] = 0.0;      // Goal omega (at rest)

    // Set start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Set the planner to RG-RRT
    //auto siPtr = ss.getSpaceInformation();
    auto planner = std::make_shared<oc::RGRRT>(ss.getSpaceInformation(),1);
    ss.setPlanner(planner);

    // Attempt to solve the planning problem
    ob::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        std::cout << "Found solution with RRT. Writing to file " << filename << std::endl;

        // Write the solution path to a file
        std::ofstream file(filename);
        if (file.is_open())
        {
            auto pathControl=ss.getSolutionPath();
            auto pathGeometric = pathControl.asGeometric();

            pathGeometric.printAsMatrix(file);  // Output solution path to a file
            file.close();
        }
        else
        {
            std::cerr << "Unable to open file for writing!" << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found." << std::endl;
    }
}



// Function for planning with KPIECE
void planPendulumWithKPIECE(double torqueLimit, const std::string &filename)
{
    // Create a state space for the pendulum (2D: [theta, omega])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set bounds for the pendulum's state space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);  // Lower bound for theta
    bounds.setHigh(0, M_PI);  // Upper bound for theta
    bounds.setLow(1, MIN_ANG_VEL);  // Lower bound for angular velocity
    bounds.setHigh(1, MAX_ANG_VEL); // Upper bound for angular velocity
    stateSpace->setBounds(bounds);

    // Define the projection evaluator for KPIECE
    stateSpace->registerProjection("PendulumProjection", ob::ProjectionEvaluatorPtr(new PendulumProjection(stateSpace.get())));

    // Create a control space for the pendulum (1D: torque)
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 1);

    // Set bounds for the control space (torque)
    ob::RealVectorBounds controlBounds(1);
    controlBounds.setLow(-torqueLimit); // Min torque
    controlBounds.setHigh(torqueLimit); // Max torque
    controlSpace->setBounds(controlBounds);

    // Create a simple setup
    oc::SimpleSetup ss(controlSpace);

    // Set the state validity checker
    ss.setStateValidityChecker([](const ob::State *state) { return isValidState(state); });

    // Set the ODE solver and system
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE);
    ss.getSpaceInformation()->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    // Define the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = -M_PI / 2; // Initial theta
    start[1] = 0.0;       // Initial omega (at rest)

    ob::ScopedState<> goal(stateSpace);
    goal[0] = M_PI / 2; // Goal theta (pendulum pointing up)
    goal[1] = 0.0;      // Goal omega (at rest)

    // Set start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Set the planner to KPIECE
    auto planner = std::make_shared<oc::KPIECE1>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Attempt to solve the planning problem
    ob::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        std::cout << "Found solution with KPIECE and torque limit " << torqueLimit << ". Writing to file " << filename << std::endl;

        // Write the solution path to a file
        std::ofstream file(filename);
        if (file.is_open())
        {
            auto pathControl=ss.getSolutionPath();
            auto pathGeometric = pathControl.asGeometric();

            pathGeometric.printAsMatrix(file);  // Output solution path to a file
            file.close();
        }
        else
        {
            std::cerr << "Unable to open file for writing!" << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found." << std::endl;
    }
}


void benchmarkPendulum()
{
    
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);
    bounds.setHigh(0, M_PI);
    bounds.setLow(1, MIN_ANG_VEL);
    bounds.setHigh(1, MAX_ANG_VEL);
    stateSpace->setBounds(bounds);

    stateSpace->registerProjection("PendulumProjection", ob::ProjectionEvaluatorPtr(new PendulumProjection(stateSpace.get())));

    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 1);
    ob::RealVectorBounds controlBounds(1);
    controlBounds.setLow(-3);
    controlBounds.setHigh(3);
    controlSpace->setBounds(controlBounds);

    oc::SimpleSetup ss(controlSpace);
    ss.setStateValidityChecker([](const ob::State *state) { return isValidState(state); });

    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE);
    ss.getSpaceInformation()->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    ob::ScopedState<> start(stateSpace);
    start[0] = -M_PI / 2;
    start[1] = 0.0;

    ob::ScopedState<> goal(stateSpace);
    goal[0] = M_PI / 2;
    goal[1] = 0.0;

    ss.setStartAndGoalStates(start, goal);

    ompl::tools::Benchmark benchmark(ss, "Pendulum Benchmark");
    benchmark.addPlanner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<oc::KPIECE1>(ss.getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<oc::RGRRT>(ss.getSpaceInformation(), 1));  // RG-RRT

    ompl::tools::Benchmark::Request request(20.0, 10000, 20);
    benchmark.benchmark(request);

    benchmark.saveResultsToFile("pendulum_benchmark_results.log");  // For Planner Arena

}


void planPendulum(int choice,double torque)
{
    switch(choice)
    {
        case 1:planPendulumwithRRT(torque,"pendulum_solution_path_RRT.txt");
            break;

        case 2:planPendulumWithKPIECE(torque,"pendulum_solution_path_KPIECE.txt");
            break;

        case 3:planPendulumwithRGRRT(torque,"pendulum_solution_path_RGRRT.txt");
            break;

    }

}


int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    //ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(planner,torque);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum();

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
