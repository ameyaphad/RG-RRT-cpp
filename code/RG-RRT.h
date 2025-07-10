 #ifndef OMPL_CONTROL_PLANNERS_RGRRT_RGRRT_
 #define OMPL_CONTROL_PLANNERS_RGRRT_RGRRT_
  
 #include <ompl/control/planners/PlannerIncludes.h>
 #include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

  
 namespace ompl
 {
     namespace control
     {
         class RGRRT : public base::Planner
         {
         public:
             RGRRT(const SpaceInformationPtr &si,int robot_);
  
             ~RGRRT() override;
  
             base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
  
             void clear() override;
  
             void setGoalBias(double goalBias)
             {
                 goalBias_ = goalBias;
             }
  
             double getGoalBias() const
             {
                 return goalBias_;
             }
  
             bool getIntermediateStates() const
             {
                 return addIntermediateStates_;
             }
  
             void setIntermediateStates(bool addIntermediateStates)
             {
                 addIntermediateStates_ = addIntermediateStates;
             }
  
             void getPlannerData(base::PlannerData &data) const override;
  
             template <template <typename T> class NN>
             void setNearestNeighbors()
             {
                 if (nn_ && nn_->size() != 0)
                     OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                 clear();
                 nn_ = std::make_shared<NN<Motion *>>();
                 setup();
             }

             
             

  
             void setup() override;
  
         protected:
             class Motion
             {
             public:
                 Motion() = default;
  
                 Motion(const SpaceInformation *si)
                   : state(si->allocState()), control(si->allocControl())
                 {
                 }
  
                 ~Motion() = default;
  
                 base::State *state{nullptr};
  
                 Control *control{nullptr};
  
                 unsigned int steps{0};
  
                 Motion *parent{nullptr};

                 std::vector<ompl::base::State *> reachableSet;
             };
  
             void freeMemory();
  
             double distanceFunction(const Motion *qnear, const Motion *qrand,ompl::base::SpaceInformationPtr si) const
             {
                 //return si_->distance(a->state, b->state);

                double minDist = std::numeric_limits<double>::infinity();
                double distQnear = si->distance(qnear->state, qrand->state);

                // Iterate over reachable states and find the closest one to qrand
                    for (const auto *qr : qnear->reachableSet) 
                    {
                        double dist = si->distance(qr, qrand->state);
                        if (dist < distQnear) 
                        {
                            minDist = std::min(minDist, dist);
                        }
                    }

                    return (minDist == std::numeric_limits<double>::infinity()) ? distQnear : minDist;
            }

            void computeReachableSet(Motion *motion, const control::SpaceInformation *si, double timeStep,int robotType) 
             {

                if(robot==1)
                {
                    std::vector<double> torques = {-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10};
                    for (double tau : torques) 
                    {
                        ompl::control::Control *control = si->allocControl();
                        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = tau;
                        
                        ompl::base::State *newState = si->allocState();
                        si->propagate(motion->state, control, timeStep, newState);
                        motion->reachableSet.push_back(newState);
                    }
                }
                else
                {
                    std::vector<double> throttleLevels = {-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0};

                    // Iterate over throttle levels to propagate and compute reachable states
                    for (double u0 : throttleLevels) 
                    {
                        // Allocate a control and set throttle level
                        ompl::control::Control *control = si->allocControl();
                        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = u0;  // Set throttle
                        // Steering (u1) is ignored here

                        // Allocate a new state to hold the result of the propagation
                        ompl::base::State *newState = si->allocState();

                        // Propagate the current state by applying the control for the specified timestep
                        si->propagate(motion->state, control, timeStep, newState);

                        // Add the resulting state to the reachable set of the motion
                        motion->reachableSet.push_back(newState);

                        // Free the allocated control (no longer needed after propagation)
                        si->freeControl(control);
                    }
                }

            }

  
             base::StateSamplerPtr sampler_;
  
             DirectedControlSamplerPtr controlSampler_;
  
             const SpaceInformation *siC_;

             control::SpaceInformationPtr *siTemp;
  
             std::shared_ptr<NearestNeighbors<Motion *>> nn_;
  
             double goalBias_{0.05};
  
             bool addIntermediateStates_{false};

             int robot;         // robot==1 stands for pendulum planning, robot==2 stands for car planning
  
             RNG rng_;
  
             Motion *lastGoalMotion_{nullptr};
         };
     }
 }
  
 #endif