/******************************************************************************
Copyright (c) 2016, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl-1.7/pcl/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>

#include "wanderer_nodelet.h"

//Nodelet dependencies
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(realsense_wanderer::WandererNodelet, nodelet::Nodelet)

namespace realsense_wanderer
{

	//******************************
	// Public Methods
	//******************************

	WandererNodelet::~WandererNodelet()
	{
		ROS_INFO_STREAM("Done - WandererNodelet");
	}

	void WandererNodelet::onInit()
	{
		ROS_INFO_STREAM("Starting Walker node");

		fillConfigMap();

		ros::NodeHandle& nh = getNodeHandle();

		image_transport::ImageTransport itColor(nh);

		mRotating = false;
		mDirection = 0.1;

		//subscribe
		mSub = nh.subscribe("camera/depth/points", 1, &WandererNodelet::pointcloudCameraCallback, this);

		//advertise
		mMarkerPub = nh.advertise<visualization_msgs::Marker>("wonderer/marker", 1);
		mGoalPub = nh.advertise<geometry_msgs::PointStamped>("wanderer/goal", 1);

		// Initialize dynamic reconfigure
		mReconfigureServer.reset(new dynamic_reconfigure::Server<realsense_wanderer::wandererConfig>(getPrivateNodeHandle()));
		mReconfigureServer->setCallback(boost::bind(&WandererNodelet::ConfigureCallback, this, _1, _2));

	}

	//===================================
	//	Member Functions
	//===================================

	void WandererNodelet::fillConfigMap()
	{
		std::vector<std::string> args = getMyArgv();
		while (args.size() > 1)
		{
			mConfig[args[0]] = args[1];
			args.erase(args.begin());
			args.erase(args.begin());
		}


		mEnabled = true;
		mMinY = 0;
		mMaxY = 20000;
		mMinX = 0;
		mMaxX = 20000;
		mMinZ = 0;
		mMaxZ = 20000;

		mMinBlobSize = 4000;


		char* key = (char*)"MinY";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinY = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MaxY";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxY = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MinX";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinX = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MaxX";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxX = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MaxZ";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxZ = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MinZ";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinZ = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MinBlobSize";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinBlobSize = atof(mConfig.at(key).c_str());
		}

	}


	bool WandererNodelet::enoughPointsDetected(const pcl::PointCloud<pcl::PointXYZ>*  cloud, float targetZ, bool freespace)
	{

		float z = 1e6;
		float x = 0.0;
		float y = 0.0;

		//Number of points observed
		unsigned int n = 0;
		//Iterate through all the points in the region and find the average of the position
		BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
		{
			//First, ensure that the point's position is valid. This must be done in a seperate
			//if because we do not want to perform comparison on a nan value.
			if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
			{
				//Test to ensure the point is within the aceptable box.
				if (pt.z > mMinZ && pt.z < targetZ && -pt.y > mMinY && -pt.y < mMaxY && (freespace || (pt.x < mMaxX && pt.x > mMinX)))
				{
					n++;
				}
			}
		}
		//If there are enough points
		return (n > mMinBlobSize);
		
	}

	// Check what direction to turn
	int WandererNodelet::calcDirection(const pcl::PointCloud<pcl::PointXYZ>*  cloud, float targetZ)
	{
		float leftDir = 0;
		float rightDir = 0;

		//Iterate through all the points in the region and find the average of the position
		BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
		{
			//First, ensure that the point's position is valid. This must be done in a seperate
			//if because we do not want to perform comparison on a nan value.
			if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
			{
				//Test to ensure the point is within the aceptable box.
				if (pt.z > mMinZ && pt.z < targetZ && -pt.y > mMinY && -pt.y < mMaxY && pt.x < mMaxX && pt.x > mMinX)
				{
					if (pt.x < 0)
						leftDir++;
					else
						rightDir++;
				}
			}
		} 

		if (leftDir > rightDir)
			return -1;
		else
			return 1;
	}


	void WandererNodelet::process(const pcl::PointCloud<pcl::PointXYZ>*  cloud)
	{

		//X,Y,Z of the centroid
		float x = 0.0;
		float y = 0.0;
		float z = 0;

		bool obstacleDetected = false;
		bool shouldReverse = false;
		bool closeObstacleDetected = false;
		if (enoughPointsDetected(cloud, MAX_FAR_DEPTH, true)) //we detect an edge in front
		{
			if (enoughPointsDetected(cloud, MAX_CLOSE_DEPTH, false)) //too close. it is an obstacle
			{
				obstacleDetected = true;
				//closeObstacleDetected = true;
				if (enoughPointsDetected(cloud, MAX_TOO_CLOSE_DEPTH, false)) //very close. it is an obstacle
				{
					closeObstacleDetected = true;
				}
			}
		}
		else
		{	//no edge. consider it as if it is an obstacle. It might be becuase we are too close.
			shouldReverse = true;
		}


		if (obstacleDetected)
		{
			if (!mRotating)
			{
				//--- Calc dirction to turn
				mDirection = ROTATION_VALUE * calcDirection(cloud, 1.0); //--- ToDo TargetZ should be from param ?

			}

			if (closeObstacleDetected)
			{
				x = mDirection * ROTATION_MLTP;
				z = 0;
			}
			else 
			{
				x = mDirection * ROTATION_MLTP;
				z = 0.3;
			}
			mRotating = true;
		}
		else
		{
			x = 0;
			z = FORWARD_VALUE;
			if (shouldReverse)
			{
				z = -FORWARD_VALUE;
			}
			mRotating = false;
		}

		
		publishMarker(x, y, z);
		publishGoal(x, y, z);


	}


	//***********************************
	// Debug/Visualize functions
	//***********************************

	void WandererNodelet::publishGoal(double x, double y, double z)
	{
		geometry_msgs::PointStamped goal;
		goal.header.frame_id = mFrame;
		goal.header.stamp = ros::Time();
		goal.point.x = x;
		goal.point.y = y;
		goal.point.z = z;
		mGoalPub.publish(goal);
	}

	void WandererNodelet::publishMarker(double x, double y, double z)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = mFrame;
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		mMarkerPub.publish(marker);

	}

	//***********************************
	// Callback functions
	//***********************************
	
	void WandererNodelet::pointcloudCameraCallback(const sensor_msgs::PointCloud2ConstPtr& msg_pointcloud)
	{

		if (mEnabled)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::fromROSMsg(*msg_pointcloud, cloud);
			int size = cloud.points.size();
			mFrame = msg_pointcloud->header.frame_id;
			process(&cloud);
		}

	}

	void WandererNodelet::ConfigureCallback(realsense_wanderer::wandererConfig &config, uint32_t level)
	{
		mMinY = config.mMinY;
		mMaxY = config.mMaxY;
		mMinX = config.mMinX;
		mMaxX = config.mMaxX;
		mMinZ = config.mMinZ;
		mMaxZ = config.mMaxZ;

		mMinBlobSize = config.mMinBlobSize;

		mEnabled = config.mEnabled;

	}
	//***********************************
	// Utils functions
	//***********************************

	bool WandererNodelet::to_bool(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);
		std::istringstream is(str);

		bool b;

		is >> std::boolalpha >> b;

		return b;
	}

	long WandererNodelet::msecTime()
	{
		struct timeval time;
		gettimeofday(&time, NULL);
		return time.tv_sec * 1000 + time.tv_usec / 1000;
	}

}

