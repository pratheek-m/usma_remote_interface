#include <walk_a_star.h>


#include <ros/ros.h>
#include <stdio.h>

// this is a byproduct of wanting to use a struct as a key for a map....
//const bool operator<(const gridPoint &r, const gridPoint &l)
//  {    return ( (r.x<l.x)&&(r.y<l.y) );  }



  void walkingAStar::init() {
    costmap_sub_ = nh_.subscribe("/travmap/travmap", 10, &walkingAStar::costmapCb, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &walkingAStar::goalCb, this); 
    cur_loc_sub_ = nh_.subscribe("cur_loc", 10, &walkingAStar::curLocationCb, this); 
    
    path_pub_ = nh_.advertise<nav_msgs::Path>("path",10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",10);
    start_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("start",10);
  }

  void walkingAStar::costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("Width: %d  ---- Height: %d ---- Resolution: %f", msg->info.width, msg->info.height, msg->info.resolution);
    o_grid_.info = msg->info;
    o_grid_.header = msg->header;
    o_grid_.data = msg->data;

    this->solve();
  }

  void walkingAStar::goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg){

    ROS_INFO("Goal Position x: %f, y:%f, z:%f Orientation x:%f, y:%f, z:%f, w:%f", msg->pose.position.x,
      msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    goal_ = msg->pose;
  }

  void walkingAStar::curLocationCb(const tf2_msgs::TFMessage::ConstPtr& msg){
    for(int i = 0; i < msg->transforms.size(); i++){
      if( 0 == strcmp((msg->transforms[i].header.frame_id).c_str(), "pelvis") ){
        cur_location_.position.x = msg->transforms[i].transform.translation.x;
        cur_location_.position.y = msg->transforms[i].transform.translation.y;
        break;
      }
    }
  }

	int walkingAStar::solve(){
    c_list_.clear();
    o_list_.clear();

    gridPoint start;
    poseToGridPoint(start, cur_location_, o_grid_.info.origin, o_grid_.info.resolution);
    ROS_INFO("START: %d,%d",start.x,start.y);
    gridPoint goal;
    poseToGridPoint(goal,goal_, o_grid_.info.origin, o_grid_.info.resolution); 
    ROS_INFO("GOAL: %d,%d",goal.x,goal.y);

    if( start.x == goal.x && start.y == goal.y ) {
       // This case exists for the goal being the start return path of size one
       c_list_.push_back(start);
        ROS_INFO("[WALKING] GOAL IS THE START");
       return 0;
    }

    o_list_.push_back(start);
    
    bool found = false;
    int count=0;
    while( found==false && !o_list_.empty() ){
      ROS_INFO("===========LOOP START==============");
      gridPoint *min = &o_list_.at(0);
      ROS_INFO("START X: %d -- Y: %d -- V: %d",min->x,min->y,min->val);
      for( int i = 1; i < o_list_.size(); i++){
        gridPoint *cur = &o_list_.at(i);
        ROS_INFO("OLIST: %d -- Y: %d -- V: %d",cur->x,cur->y,moveComparitor(goal, min, cur ) );
        if( moveComparitor(goal, min, cur ) < 0)
          min = &o_list_.at(i);
      }

      ROS_INFO("MIN X: %d -- Y: %d -- V: %d",min->x,min->y,min->val);
      c_list_.push_back(*min);
      int cx = c_list_.back().x;
      int cy = c_list_.back().y;

      o_list_.clear();
      for(int i = 0; i < 4; i++){
//        ROS_INFO("%d %d", cx+i%2, cy+i/2);
        gridPoint tmp;
        int dx, dy;
        switch(i){
          case 0: dx=-1; dy=0; break;
          case 1: dx=1; dy=0; break;
          case 2: dx=0; dy=-1; break;
          case 3: dx=0; dy=1; break;
        }
        tmp.x = cx+dx;
        tmp.y = cy+dy;
        tmp.val =  (int)(o_grid_.data[(cx+dx)*o_grid_.info.width+(cy+dy)]);
//        ROS_INFO("NEIGHTBORS X: %d -- Y: %d -- V: %d",tmp.x,tmp.y,tmp.val);
        // need to set this threshold as a single variable somewhere
        bool exists=false;
        for( int j = 0; j<c_list_.size(); j++){
          if( tmp.x == c_list_.at(j).x  && tmp.y == c_list_.at(j).y ){
             exists = true;
          }
        }
        if( tmp.val<50 && !exists && tmp.val!=-1 )
          o_list_.push_back( tmp );
      }
      // checks for goal in the closed list
      if( c_list_.back().x == goal.x && c_list_.back().y == goal.y ){
        found = true;
      }
      if (count++>=50)
        found = true;
    }
    ROS_INFO("[WALKING] PLAN FOUND");
  //  gridPoint *tmp;
  //  for( int i = 0; i < c_list_.size(); i++){
  //    tmp = &c_list_.at(i);
  //    ROS_INFO("%d: X: %d -- Y: %d -- V: %d",i,tmp->x,tmp->y,tmp->val);
  //  }
    listToPath(path_, o_grid_.info.origin, o_grid_.info.resolution, c_list_);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    gridPointToPose(pose.pose,goal,o_grid_.info.origin,o_grid_.info.resolution);
    goal_pub_.publish(pose);
    gridPointToPose(pose.pose,start,o_grid_.info.origin,o_grid_.info.resolution);
    start_pub_.publish(pose);

    path_pub_.publish(path_);
    return 0;
  }
 
  void walkingAStar::listToPath(nav_msgs::Path& path, geometry_msgs::Pose& origin, float resolution, std::vector<gridPoint>& list){
    path.poses.clear();
    geometry_msgs::PoseStamped pose;
    path.header.frame_id="map";
    path.header.seq++;
    for( int i = 0; i < list.size(); i++ ){
      gridPointToPose(pose.pose, list.at(i), origin, resolution);
      pose.header.frame_id="map";
      pose.header.seq=i;
      path.poses.push_back(pose);
    }
  }

  void walkingAStar::poseToGridPoint(gridPoint& delta, geometry_msgs::Pose& pose, geometry_msgs::Pose& origin, float resolution){
    delta.x = (pose.position.x-origin.position.x)/resolution;
    delta.y = (pose.position.y-origin.position.y)/resolution;
    delta.val = 0;
  }
  void walkingAStar::gridPointToPose(geometry_msgs::Pose& pose, gridPoint& point, geometry_msgs::Pose& origin, float resolution){
    pose.position.x = ((float)point.x)*resolution + origin.position.x;
    pose.position.y = ((float)point.y)*resolution + origin.position.y;
  }

  int walkingAStar::moveComparitor(gridPoint& goal, gridPoint* one, gridPoint* two){
    // returns 0 for equal, -1 for one better than two, 1 for two better than one


    // keeps track of the deltas
    int dxo,dyo,dxt,dyt,score_one,score_two;
    dxo = one->x-goal.x;
    dyo = one->y-goal.y;
    dxt = two->x-goal.x;
    dyt = two->y-goal.y;
   
    score_one = 5*sqrt((float)(dxo*dxo+dyo*dyo))+one->val; 
    score_two = 5*sqrt((float)(dxt*dxt+dyt*dyt))+two->val; 
    return score_two-score_one;
  }

int main(int argc, char **argv)
{

  ros::init(argc,argv,"walk_planner");

  walkingAStar walk;
  walk.init();
  ros::spin();
  return 0;
}
