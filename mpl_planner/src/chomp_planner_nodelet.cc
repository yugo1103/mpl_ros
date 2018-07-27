#include "mpl_planner/MPLPlannerNode.hpp"
#include <nodelet/nodelet.h>

namespace mpl_planner_nodelet {

class MPLPlannerNodelet : public nodelet::Nodelet
{
private:
  mpl_planner::MPLPlannerNode *mplPlannerNode;

public:
  MPLPlannerNodelet() : Nodelet(), mplPlannerNode(NULL)
  {
  }

  ~MPLPlannerNodelet()
  {
    if(mplPlannerNode)
    {
      delete mplPlannerNode;
    }
  }

  virtual void onInit()
  {
    mplPlannerNode = new mpl_planner::MPLPlannerNode(getNodeHandle(), getPrivateNodeHandle());
  }
};
}  // namespace chomp_planner_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpl_planner_node::MPLPlannerNodelet, nodelet::Nodelet)

