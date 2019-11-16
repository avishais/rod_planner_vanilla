

#include "plan.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
        // case PLANNER_CBIRRT:
        // {
        //     return std::make_shared<og::CBiRRT>(si, maxStep);
        //     break;
        // }
        // case PLANNER_RRT:
        // {
        //     return std::make_shared<og::RRT>(si, maxStep);
        //     break;
        // }
        // /*case PLANNER_LAZYRRT:
        // {
        //     return std::make_shared<og::LazyRRT>(si, maxStep);
        //     break;
        // }*/
        // case PLANNER_PRM:
        // {
        //     return std::make_shared<og::PRM>(si);
        //     break;
        // }
        // case PLANNER_SBL:
        // {
        //     return std::make_shared<og::SBL>(si, maxStep);
        //     break;
        // }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

int main(int argn, char ** args) {
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime; // Maximum allowed runtime
	plannerType ptype; // Planner type
	string plannerName;
	int env; // Tested environment index
}