/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <limits>
#include <pluginlib/class_list_macros.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// pcl filter
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>  // pcl::transformPointCloud
// openmp
#include <omp.h>

namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet()
{
}

void PointCloudToLaserScanNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  // filter by inclination angle in radian relative to 
  // the origin of camera center project to the ground plan
  // radians angle upward, the rotation (pitch of certain degrees around the Y-axis)
  private_nh_.param<double>("inclination_angle", inclination_angle_, 0.0);

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  // the x offset between base_link origin and camera center, according to description file 0.91 m
  private_nh_.param<double>("tf_x_offset", tf_x_offset_, 0.91);

  // the base_link z axis offset relative to the flat ground plane, we suppose base_link xOy plane is parallel with flat ground.
  // in this case is the radius of rear wheel 0.25 m
  private_nh_.param<double>("tf_z_offset", tf_z_offset_, 0.25);

  // set bool flag for pointcloud_filtered_pub_ enable debug mode
  private_nh_.param<bool>("enable_debug_mode", enable_debug_mode_, false);

  // robot frame name, by default "base_link"
  private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }

  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                               boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));

  // segmentation of point cloud using z axis height relative to flat ground
  // this is debug topic for visualization of filtered point cloud
  if (enable_debug_mode_) {
    pointcloud_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_filter", 1000, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                                boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  }

  // set offset, the base_link z axis offset relative to the falt ground, we suppose base_link xOy plane is parallel with flat ground.
  ROS_INFO("[pointcloud_to_laserscan_nodelet] origin min_height = %f, origin max_height = %f", min_height_, max_height_);
  min_height_ -= tf_z_offset_;
  max_height_ -= tf_z_offset_;
  ROS_INFO("[pointcloud_to_laserscan_nodelet] tf_z_offset = %f, offset min_height = %f, offset max_height = %f, inclination_angle = %f", tf_z_offset_, min_height_, max_height_, inclination_angle_);
}

void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
    sub_.subscribe(nh_, "cloud_in", input_queue_size_);
  }
}

void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    sub_.unsubscribe();
  }
}

void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header = cloud_msg->header;
  if (!target_frame_.empty())
  {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
  if (!(output.header.frame_id == cloud_msg->header.frame_id))
  {
    try
    {
      cloud.reset(new sensor_msgs::PointCloud2);
      // transform all point clouds to the same coordinate system under base_link frame
      // this is easy to trim/segment different point clouds in the same frame
      tf2_->transform(*cloud_msg, *cloud, robot_frame_, ros::Duration(tolerance_));
      cloud_out = cloud;
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    cloud_out = cloud_msg;
  }

  // Convert the transformed point cloud ROS PointCloud2 message to a PCL PointCloud
  // Create a new PCL PointCloud for the filtered points
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_out, *filtered_pcl_cloud);

  // Create the passthrough filter
  pcl::PassThrough<pcl::PointXYZ> pass;

  //----------------------------------------------------------
  // angle inclination filter
  //----------------------------------------------------------

  // Step 1: Translate the point cloud so base_point becomes the origin (0.91, 0, min_height_) camera center project to ground plane
  Eigen::Affine3f min_translation_to_origin = Eigen::Affine3f::Identity();
  min_translation_to_origin.translation() << -tf_x_offset_, 0.0, -min_height_;

  // Step 2: Define the rotation (pitch of certain degrees around the Y-axis)
  Eigen::Affine3f pitch_rotation_transform = Eigen::Affine3f::Identity();
  pitch_rotation_transform.rotate(Eigen::AngleAxisf(inclination_angle_, Eigen::Vector3f::UnitY()));

  // Step 3: Translate back to the original position (0.91, 0, min_height_)
  Eigen::Affine3f min_translation_back = Eigen::Affine3f::Identity();
  min_translation_back.translation() << tf_x_offset_, 0.0, min_height_;

  // Step 4: transformation & Transform the point cloud
  Eigen::Affine3f min_transform = min_translation_back * pitch_rotation_transform * min_translation_to_origin;
  pcl::transformPointCloud(*filtered_pcl_cloud, *filtered_pcl_cloud, min_transform);

  // Step 5: Apply PassThrough filter in the Z-axis (height) in the transformed frame
  pass.setInputCloud(filtered_pcl_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_height_, std::numeric_limits<float>::max()); // keep points above the plane
  pass.filter(*filtered_pcl_cloud);

  // Step 6: Transform back to the original frame
  Eigen::Affine3f min_inverse_transform = min_transform.inverse();
  pcl::transformPointCloud(*filtered_pcl_cloud, *filtered_pcl_cloud, min_inverse_transform);

  // Step 7: Translate the point cloud so base_point becomes the origin (0.91, 0, max_height) camera center project to ground plane
  Eigen::Affine3f max_translation_to_origin = Eigen::Affine3f::Identity();
  max_translation_to_origin.translation() << -tf_x_offset_, 0.0, -max_height_;

  // Step 8: Translate back to the original position (0.91, 0, max_height)
  Eigen::Affine3f max_translation_back = Eigen::Affine3f::Identity();
  max_translation_back.translation() <<  tf_x_offset_, 0, max_height_;

  // Step 9: transformation & Transform the point cloud
  Eigen::Affine3f max_transform = max_translation_back * pitch_rotation_transform * max_translation_to_origin;
  pcl::transformPointCloud(*filtered_pcl_cloud, *filtered_pcl_cloud, max_transform);

  // Step 10: Apply PassThrough filter in the Z-axis (height) in the transformed frame
  pass.setInputCloud(filtered_pcl_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-std::numeric_limits<float>::max(), max_height_); // keep points above the plane
  pass.filter(*filtered_pcl_cloud);

   // Step 11: Transform back to the original frame
  Eigen::Affine3f max_inverse_transform = max_transform.inverse();
  pcl::transformPointCloud(*filtered_pcl_cloud, *filtered_pcl_cloud, max_inverse_transform);

  // Convert the filtered PCL PointCloud back to a ROS PointCloud2 message
  sensor_msgs::PointCloud2 ros_output_cloud;
  pcl::toROSMsg(*filtered_pcl_cloud, ros_output_cloud);

  // Set the header of the output message with robot_frame name
  ros_output_cloud.header = cloud_msg->header;
  ros_output_cloud.header.frame_id = robot_frame_;

  // Publish the filtered point cloud
  // if debug mode enable flag is true
  if (enable_debug_mode_) {
    pointcloud_filtered_pub_.publish(ros_output_cloud);
  }

  // transform the filtered point cloud from robot frame to target frame after trimming/segmentation/filtering 
  try {
    cloud.reset(new sensor_msgs::PointCloud2);
    tf2_->transform(ros_output_cloud, *cloud, target_frame_, ros::Duration(tolerance_));
    cloud_out = cloud;
  }
  catch (tf2::TransformException& ex) {
    NODELET_ERROR_STREAM("Transform failure: " << ex.what());
    return;
  }

  // Iterate through pointcloud
  // Parallelize this section using OpenMP
  #pragma omp parallel for
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  pub_.publish(output);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)
