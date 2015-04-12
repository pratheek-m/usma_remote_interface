#ifndef WALK_A_STAR
#define WALK_A_STAR

//ROS
#include<ros/ros.h>

//OccupancyGrid
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

//Path Message
#include <nav_msgs/Path.h>

//Robot Pose through TF
#include <tf2_msgs/TFMessage.h>

//Goal Message
#include <geometry_msgs/PoseStamped.h>

struct gridPoint {
  int x, y, val;

};

/**

  Solves for currently known terrain. The destination allows for a general direction to be made.

  However, the solver can only solve up to the edge of the known area. Unknown will be treated as a untraversable. To complete the path from current location to the destination a straight line will be made from the edge to the destination.

**/

  class walkingAStar {
    public:
      void init();
      void costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
      void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void curLocationCb(const tf2_msgs::TFMessage::ConstPtr& msg);
    private:

      ros::NodeHandle nh_;
      ros::Subscriber costmap_sub_;
      ros::Subscriber goal_sub_;
      ros::Subscriber cur_loc_sub_;
      ros::Publisher goal_pub_;
      ros::Publisher start_pub_;
      ros::Publisher path_pub_;
    
      geometry_msgs::Pose cur_location_;
      geometry_msgs::Pose goal_;
      nav_msgs::OccupancyGrid o_grid_;

      std::vector<gridPoint> c_list_;
      std::vector<gridPoint> o_list_;
      std::vector<gridPoint> obstacle_list_;

      nav_msgs::Path path_;

      /** solves for a path along traversability map--works in array coordinates
      if succeed returns 0--if fail return how far it got **/
      int solve();
      int moveComparitor(gridPoint& goal, gridPoint* one, gridPoint* two);

      // converts the open list to a 2D path
      void listToPath(nav_msgs::Path& path, geometry_msgs::Pose& origin, float resolution, std::vector<gridPoint>& list);

      // converts a pose to the array indicies where it will be found
      void poseToGridPoint(gridPoint& delta, geometry_msgs::Pose& pose, geometry_msgs::Pose& origin, float resolution);
      void gridPointToPose(geometry_msgs::Pose& pose, gridPoint& point, geometry_msgs::Pose& origin, float resolution);
  };

#endif
