#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
void PublishCloud(ros::Publisher& cloud_pub)
{
    sensor_msgs::PointCloud2 cloud;
   unsigned int num =100;
     cloud.header.stamp = ros::Time::now();
     cloud.header.frame_id = "robot";

     cloud.point.resize(num);
     cloud.channels.resize(1);
     cloud.channels[0].name= "intensities";
for(int i=0;i<num;i++)
{
    cloud.points[i].x=rang[i]*cos(angle[i]);
    cloud.points[i].y=rang[i]*sin(angle[i]);
    cloud.points[i].z= 0;
    cloud.channels[i].values[i]=100 ;
}
    cloud_pub.publish(cloud);
    ros::Duration(1.0);
}

int main(int argc, char * argv[])
 {

 ros::init(argc, argv, "Map");
 ros::NodeHandle nh;
 ros::NodeHandle nh_private("~");
 ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("Cloud",1,true);
 ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/Map", 1); 
 nav_msgs::OccupancyGrid map;
 map.header.frame_id="grid"; 
 map.header.stamp = ros::Time::now(); 
 map.info.resolution = 0.5; 
 float32 map.info.width = 10; 
 map.info.height     = 10;  

   for(int i=0;i<10;i++)
 {
   for(int j;j<10;j++)
   {

      int p[i][j] = 0;   // [0,100]
}
}
  p[0][0] = 100;
  p[1][0] = 100;
  std::vector<signed char> a(p, p+100);
  map.data = a;

  while (ros::ok())
  {
      pub.publish(map);
  }

  ros::shutdown();
  return 0;
}


