#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/ODESolver.h>
#include <Eigen/Core> // For Eigen::VectorXd
#include <iostream>
#include <fstream> // For writing solution to file
#include "CollisionChecking.h"
#include <ompl/tools/benchmark/Benchmark.h>
#include "RG-RRT.h"
#include <ompl/geometric/PathGeometric.h>


// Define the namespaces for ompl::base and ompl::control
namespace ob = ompl::base;
namespace oc = ompl::control;


// Projection for the KPIECE planner
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space) {}

    unsigned int getDimension() const override
    {
        return 3; // Project onto x, y, and theta
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *carState = state->as<ob::RealVectorStateSpace::StateType>();
        projection[0] = carState->values[0]; // x
        projection[1] = carState->values[1]; // y
        projection[2] = carState->values[2]; // theta
    }
};

// Define the car's ODE
void carODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    const double x = q[0];
    const double y = q[1];
    const double theta = q[2];
    const double v = q[3];

    // Extract control inputs (omega and v_dot)
    const double omega = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    const double v_dot = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];

    // Set up the system of ODEs
    qdot.resize(4);
    qdot[0] = v * cos(theta); // x_dot = v * cos(theta)
    qdot[1] = v * sin(theta); // y_dot = v * sin(theta)
    qdot[2] = omega;          // theta_dot = omega
    qdot[3] = v_dot;          // v_dot = forward acceleration (from control)
}


// Define the obstacles (street environment)
void makeStreet(std::vector<Rectangle> &obstacles)
{
    Rectangle building1 = {5.0, 5.0, 2.0, 3.0};  // x, y, width, height
    Rectangle building2 = {10.0, 8.0, 3.0, 3.0};

    obstacles.push_back(building1);
    obstacles.push_back(building2);
}



// Define the state validity checker for the car
bool isCarStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    const auto *carState = state->as<ompl::base::RealVectorStateSpace::StateType>();

    double x = carState->values[0];
    double y = carState->values[1];
    double theta = carState->values[2];
    double v = carState->values[3];

    // Check if the car's velocity is within bounds
    if (v < -10.0 || v > 10.0)
        return false;

    // Check for collisions with obstacles
    return isValidSquare(x, y,theta,0.3,obstacles);
}




// Function for planning with RRT
void planCarwithRRT(const std::string &filename)
{
    // Create a state space for the car (4D: [x, y, theta, v])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(4);

    // Set bounds for the state space
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, 0.0);     // x lower bound
    bounds.setHigh(0, 15.0);   // x upper bound
    bounds.setLow(1, 0.0);     // y lower bound
    bounds.setHigh(1, 15.0);   // y upper bound
    bounds.setLow(2, -M_PI);   // theta lower bound
    bounds.setHigh(2, M_PI);   // theta upper bound
    bounds.setLow(3, -10.0);   // velocity lower bound
    bounds.setHigh(3, 10.0);   // velocity upper bound
    stateSpace->setBounds(bounds);

    // Create a control space for the car (2D: [omega, v_dot])
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2);

    // Set bounds for the control space
    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1.0);   // omega lower bound
    controlBounds.setHigh(0, 1.0);   // omega upper bound
    controlBounds.setLow(1, -1.0);   // v_dot lower bound
    controlBounds.setHigh(1, 1.0);   // v_dot upper bound
    controlSpace->setBounds(controlBounds);

    // Set up the SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // Capture 'ss' and 'obstacles' in the lambda
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    ss->setStateValidityChecker([ss, obstacles](const ompl::base::State *state) {
        return isCarStateValid(ss->getSpaceInformation().get(), state, obstacles);
    });

    // Set the ODE solver
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->getSpaceInformation()->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Set the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = 1.0;  // Start x
    start[1] = 1.0;  // Start y
    start[2] = 0.0;  // Start theta (angle)
    start[3] = 0.0;  // Start velocity

    ob::ScopedState<> goal(stateSpace);
    goal[0] = 10.0;  // Goal x
    goal[1] = 10.0;  // Goal y
    goal[2] = 0.0;   // Goal theta (angle)
    goal[3] = 0.0;   // Goal velocity

    ss->setStartAndGoalStates(start, goal);

    // Set the planner to RRT
    auto planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    // Solve the planning problem
    ob::PlannerStatus solved = ss->solve(20.0);

    if (solved)
    {
        std::cout << "Found solution with RRT. Writing to file " << filename << std::endl;

        // Write the solution path to a file
        std::ofstream file(filename);
        if (file.is_open())
        {
            auto pathControl=ss->getSolutionPath();
            auto pathGeometric = pathControl.asGeometric();

            file<<"SE2 0.3"<<"\n";
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


void planCarwithRGRRT(const std::string &filename)
{
    // Create a state space for the car (4D: [x, y, theta, v])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(4);

    // Set bounds for the state space
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, 0.0);     // x lower bound
    bounds.setHigh(0, 15.0);   // x upper bound
    bounds.setLow(1, 0.0);     // y lower bound
    bounds.setHigh(1, 15.0);   // y upper bound
    bounds.setLow(2, -M_PI);   // theta lower bound
    bounds.setHigh(2, M_PI);   // theta upper bound
    bounds.setLow(3, -10.0);   // velocity lower bound
    bounds.setHigh(3, 10.0);   // velocity upper bound
    stateSpace->setBounds(bounds);

    // Create a control space for the car (2D: [omega, v_dot])
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2);

    // Set bounds for the control space
    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1.0);   // omega lower bound
    controlBounds.setHigh(0, 1.0);   // omega upper bound
    controlBounds.setLow(1, -1.0);   // v_dot lower bound
    controlBounds.setHigh(1, 1.0);   // v_dot upper bound
    controlSpace->setBounds(controlBounds);

    // Set up the SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // Capture 'ss' and 'obstacles' in the lambda
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    ss->setStateValidityChecker([ss, obstacles](const ompl::base::State *state) {
        return isCarStateValid(ss->getSpaceInformation().get(), state, obstacles);
    });

    // Set the ODE solver
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->getSpaceInformation()->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Set the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = 1.0;  // Start x
    start[1] = 1.0;  // Start y
    start[2] = 0.0;  // Start theta (angle)
    start[3] = 0.0;  // Start velocity

    ob::ScopedState<> goal(stateSpace);
    goal[0] = 10.0;  // Goal x
    goal[1] = 10.0;  // Goal y
    goal[2] = 0.0;   // Goal theta (angle)
    goal[3] = 0.0;   // Goal velocity

    ss->setStartAndGoalStates(start, goal);

    // Set the planner to RRT
    auto planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation(),2);
    ss->setPlanner(planner);

    // Solve the planning problem
    ob::PlannerStatus solved = ss->solve(20.0);

    if (solved)
    {
        std::cout << "Found solution with RRT. Writing to file " << filename << std::endl;

        // Write the solution path to a file
        std::ofstream file(filename);
        if (file.is_open())
        {
            auto pathControl=ss->getSolutionPath();
            auto pathGeometric = pathControl.asGeometric();

            file<<"SE2 0.3"<<"\n";
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
void planCarWithKPIECE(const std::string &filename)
{
    // Create a state space for the car (4D: [x, y, theta, v])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(4);

    // Set bounds for the state space
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, 0.0);     // x lower bound
    bounds.setHigh(0, 15.0);   // x upper bound
    bounds.setLow(1, 0.0);     // y lower bound
    bounds.setHigh(1, 15.0);   // y upper bound
    bounds.setLow(2, -M_PI);   // theta lower bound
    bounds.setHigh(2, M_PI);   // theta upper bound
    bounds.setLow(3, -10.0);   // velocity lower bound
    bounds.setHigh(3, 10.0);   // velocity upper bound
    stateSpace->setBounds(bounds);

    // Define the projection evaluator for KPIECE
    stateSpace->registerProjection("CarProjection", ob::ProjectionEvaluatorPtr(new CarProjection(stateSpace.get())));

    // Create a control space for the car (2D: [omega, v_dot])
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2);

    // Set bounds for the control space
    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1.0);   // omega lower bound
    controlBounds.setHigh(0, 1.0);   // omega upper bound
    controlBounds.setLow(1, -1.0);   // v_dot lower bound
    controlBounds.setHigh(1, 1.0);   // v_dot upper bound
    controlSpace->setBounds(controlBounds);

    // Set up the SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // Capture 'ss' and 'obstacles' in the lambda
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    ss->setStateValidityChecker([ss, obstacles](const ompl::base::State *state) {
        return isCarStateValid(ss->getSpaceInformation().get(), state, obstacles);
    });

    // Set the ODE solver
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->getSpaceInformation()->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Set the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = 1.0;  // Start x
    start[1] = 1.0;  // Start y
    start[2] = 0.0;  // Start theta (angle)
    start[3] = 0.0;  // Start velocity

    ob::ScopedState<> goal(stateSpace);
    goal[0] = 10.0;  // Goal x
    goal[1] = 10.0;  // Goal y
    goal[2] = 0.0;   // Goal theta (angle)
    goal[3] = 0.0;   // Goal velocity

    ss->setStartAndGoalStates(start, goal);

    // Set the planner to KPIECE
    auto planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    // Solve the planning problem
    ob::PlannerStatus solved = ss->solve(20.0);

    if (solved)
    {
        std::cout << "Found solution with KPIECE. Writing to file " << filename << std::endl;

        // Write the solution path to a file
        std::ofstream file(filename);
        if (file.is_open())
        {
            auto pathControl=ss->getSolutionPath();
            auto pathGeometric = pathControl.asGeometric();

            file<<"SE2 0.3"<<"\n";
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


void benchmarkCar()
{
    
    // Create a state space for the car (4D: [x, y, theta, v])
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(4);

    // Set bounds for the state space
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, 0.0);     // x lower bound
    bounds.setHigh(0, 15.0);   // x upper bound
    bounds.setLow(1, 0.0);     // y lower bound
    bounds.setHigh(1, 15.0);   // y upper bound
    bounds.setLow(2, -M_PI);   // theta lower bound
    bounds.setHigh(2, M_PI);   // theta upper bound
    bounds.setLow(3, -10.0);   // velocity lower bound
    bounds.setHigh(3, 10.0);   // velocity upper bound
    stateSpace->setBounds(bounds);

    // Define the projection evaluator for KPIECE
    stateSpace->registerProjection("CarProjection", ob::ProjectionEvaluatorPtr(new CarProjection(stateSpace.get())));

    // Create a control space for the car (2D: [omega, v_dot])
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2);

    // Set bounds for the control space
    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1.0);   // omega lower bound
    controlBounds.setHigh(0, 1.0);   // omega upper bound
    controlBounds.setLow(1, -1.0);   // v_dot lower bound
    controlBounds.setHigh(1, 1.0);   // v_dot upper bound
    controlSpace->setBounds(controlBounds);

    // Set up the SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // Capture 'ss' and 'obstacles' in the lambda
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    ss->setStateValidityChecker([ss, obstacles](const ompl::base::State *state) {
        return isCarStateValid(ss->getSpaceInformation().get(), state, obstacles);
    });

    // Set the ODE solver
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->getSpaceInformation()->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Set the start and goal states
    ob::ScopedState<> start(stateSpace);
    start[0] = 1.0;  // Start x
    start[1] = 1.0;  // Start y
    start[2] = 0.0;  // Start theta (angle)
    start[3] = 0.0;  // Start velocity

    ob::ScopedState<> goal(stateSpace);
    goal[0] = 10.0;  // Goal x
    goal[1] = 10.0;  // Goal y
    goal[2] = 0.0;   // Goal theta (angle)
    goal[3] = 0.0;   // Goal velocity

    ss->setStartAndGoalStates(start, goal);

    ompl::tools::Benchmark b(*ss, "Car");

    b.addExperimentParameter("time", "REAL", "20.0");

    // Set up planners
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation(),2)); // Assuming RG-RRT is implemented
    b.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));

    // Run the benchmark
    ompl::tools::Benchmark::Request request(20.0, 10000, 20); // Time limit, max states, runs
    b.benchmark(request);
    b.saveResultsToFile("car_benchmark.log");

}

void planCar(int choice)
{
    switch(choice)
    {
        case 1:planCarwithRRT("car_solution_path_RRT.txt");
            break;

        case 2:planCarWithKPIECE("car_solution_path_KPIECE.txt");
            break;

        case 3:planCarwithRGRRT("car_solution_path_RGRRT.txt");
            break;

    }

}


int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    //ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        planCar(planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar();

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
