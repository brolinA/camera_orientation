#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "pcl/PointIndices.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include <pcl/filters/crop_box.h>
#include<cmath>
#include<math.h>


ros::Publisher floor_cloud_pub, wall_cloud_pub, stable_cloud_pub, yaw_angle, pitch_angle;
geometry_msgs::PointStamped pitch, yaw;
bool find_pitch, find_yaw;

double angle_measurement(pcl::PointCloud<pcl::PointXYZ>::Ptr agg_cloud_pcl, int i)
{

    pcl::PointIndices::Ptr indices(new pcl::PointIndices());

    pcl::PointIndices indices_internal;
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		// Search for a plane perpendicular to some axis (specified below).
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		// Set the distance to the plane for a point to be an inlier.
		seg.setDistanceThreshold(0.005);
		seg.setInputCloud(agg_cloud_pcl);

		
		/*
		Note:
		This axis is with respect to the camera_depth_optical_frame of the camera which
		is z-axis points outside the camera and y-axis pointing towards the ground fronm
		the center of the IR emitter.

		To find pitch of the camera, find the plane that is perpendicular to y-axis of 
		camera_depth_optical_frame i.e, the floor
			use " axis << 0, 0, 1; "

		To find yaw of the camera, find the plane that is perpendicular to z-axis of 
		camera_depth_optical_frame i.e, the wall in front of the camera
			use " axis << 0, 1, 0; "

		*/

		Eigen::Vector3f axis;
		
		if (i == 1)
			axis << 0, 0, 1;
		else
			axis << 0, 1, 0;
		
		seg.setAxis(axis);
		seg.setEpsAngle(pcl::deg2rad(10.0));

		// coeff contains the coefficients of the plane:
		// ax + by + cz + d = 0
		pcl::ModelCoefficients coeff;
		seg.segment(indices_internal, coeff);

		double a, b, c;

		a = coeff.values[0];
		b = coeff.values[1];
		c = coeff.values[2];

		//ROS_INFO("Coefficients are : %lf %lf %lf ", a, b, c);

		//angle between two planes taking a plane which is in x-y plane
		//i.e, having normal from the plane as z-axis.
		double slope, rad_, deg_;

		slope = (c)/sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

    rad_ = acos(slope);

		deg_ = rad_ * 180/3.1415; 

	//	ROS_INFO("Angle : %1.3lf ", deg_);

		*indices = indices_internal;

		if (indices->indices.size() == 0) {
		  ROS_ERROR("Unable to find surface.");
		  //return;
			}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Extract subset of original_cloud into subset_cloud:
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(agg_cloud_pcl);
	extract.setIndices(indices);
	extract.filter(*subset_cloud);

	if (i ==1)
	{
    wall_cloud_pub.publish(*subset_cloud);
		yaw.header.stamp = ros::Time::now();
		yaw.point.x = deg_;
		yaw_angle.publish(yaw);
	}
	else
	{
    floor_cloud_pub.publish(*subset_cloud);
		pitch.header.stamp = ros::Time::now();
    pitch.point.x = deg_;
		pitch_angle.publish(pitch);
	}
  return rad_;
}

void CameraCallback(const sensor_msgs::PointCloud2& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr yaw_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pitch_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr stable_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::CropBox < pcl::PointXYZ > yaw_filter;
	yaw_filter.setMin(Eigen::Vector4f(-1.0, -0.5, -0.7, 1.0));
	yaw_filter.setMax(Eigen::Vector4f(1.0, 0.5, 1.5, 1.0));
	yaw_filter.setInputCloud(cloud);
	yaw_filter.filter(*yaw_cloud_pcl);

	pcl::CropBox < pcl::PointXYZ > pitch_filter;
	pitch_filter.setMin(Eigen::Vector4f(-1.0, -0.05, -0.7, 1.0));
	pitch_filter.setMax(Eigen::Vector4f(1.0, 0.5, 1.5, 1.0));
	pitch_filter.setInputCloud(cloud);
	pitch_filter.filter(*pitch_cloud_pcl);

  double pitch_angle_;

	if(find_pitch)
	  pitch_angle_ = angle_measurement(pitch_cloud_pcl,2);

	if (find_yaw)
		angle_measurement(yaw_cloud_pcl,1);

	pcl::CropBox < pcl::PointXYZ > final_filter;
  final_filter.setMin(Eigen::Vector4f(-1.0, -0.5, -1.5, 1.0));
  final_filter.setMax(Eigen::Vector4f(5.0, 0.05, 5.0, 1.0));
  final_filter.setInputCloud(cloud);
  final_filter.setRotation(Eigen::Vector3f(-(1.57-pitch_angle_), 0, 0));
  final_filter.filter(*stable_cloud_pcl);
  stable_cloud_pub.publish(*stable_cloud_pcl);
}

void load_parameters(ros::NodeHandle nh_)
{
	nh_.getParam("/find_pitch", find_pitch);
	nh_.getParam("/find_yaw", find_yaw);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "camera_orientation_node");
  ros::NodeHandle nh;
	load_parameters(nh);
  floor_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("floor_plane", 1, true);
  wall_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_plane", 1, true);
  yaw_angle = nh.advertise<geometry_msgs::PointStamped>("yaw_angle",1,true);
  stable_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("stabilized_cloud", 1, true);
  pitch_angle = nh.advertise<geometry_msgs::PointStamped>("pitch_angle",1,true);
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, CameraCallback);
  ros::spin();
  return 0;
}
