#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> sor;
  sor.setFilterFieldName("z");
  sor.setFilterLimits(0.05,10.0);
  sor.setInputCloud (msg);
  PointCloud cloud_filtered;
  sor.filter (cloud_filtered);
  printf ("Filtered Cloud: width = %d, height = %d\n", cloud_filtered.width, cloud_filtered.height);
  pub.publish( cloud_filtered );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vcpp");
  ros::NodeHandle nh;
  pub = nh.advertise<PointCloud> ("/vcpp/cloud_out",1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/vcpp/cloud_in", 1, callback);
  ros::spin();
}
