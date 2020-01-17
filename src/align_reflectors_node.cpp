#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <align_reflectors/AlignReflectorsAction.h>

class AlignReflectorsAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<align_reflectors::AlignReflectorsAction> actionServer_; 
  std::string action_name_;
//   actionlib_tutorials::FibonacciFeedback feedback_;
  align_reflectors::AlignReflectors result_;

public:

  AlignReflectorsAction(std::string name) :
    actionServer_(nh_, name, boost::bind(&AlignReflectorsAction::executeCB, this, _1), false),
    action_name_(name)
  {
    actionServer_.start();
  }

  ~AlignReflectorsAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::AlignReflectorsGoalConstPtr &goal)
  {
    ros::Rate r(1);
    bool success = true;

    // feedback_.sequence.clear();
    // feedback_.sequence.push_back(0);
    // feedback_.sequence.push_back(1);

    // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    for(int i=1; i<=goal->order; i++)
    {
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        actionServer_.setPreempted();
        success = false;
        break;
      }
    //   feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
    //   actionServer_.publishFeedback(feedback_);
      
      r.sleep();
    }

    if(success)
    {
      result_.outcome = success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      actionServer_.setSucceeded(result_);
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_reflectors");

  AlignReflectorsAction alignReflectors("align_reflectors");
  ros::spin();

  return 0;
}