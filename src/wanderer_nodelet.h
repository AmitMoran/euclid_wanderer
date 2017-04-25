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

#pragma once
# ifndef WANDERER_NODELET
# define WANDERER_NODELET

///////////////////////////////////////////////
/// Dependencies
///////////////////////////////////////////////

#include <nodelet/nodelet.h>
#include <std_msgs/Float32.h>


#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

#include "realsense_wanderer/wandererConfig.h"


namespace realsense_wanderer
{
	///////////////////////////////////////////////
	///	WandererNodelet -
	///		This nodelet register to pointcloud messages and publishes a goal for the robot movement control to handle.
	///			the algorithm is that the robot should advance as long as it sees there is where to advance. if an obstacle 
	///			is detected, the robot will find the direction in which it is most likely not to have obstacle, and will continue there.
	///			If the robot is too close to an obstacle, it will reverse. 
	///////////////////////////////////////////////
	class WandererNodelet : public nodelet::Nodelet
	{
	public :

		const float MAX_FAR_DEPTH = 3.5; 
		const float MAX_CLOSE_DEPTH = 1.3;
		const float MAX_TOO_CLOSE_DEPTH = 0.7;
		const float ROTATION_VALUE = 0.1;
		const float ROTATION_MLTP = 0.8;
		const float FORWARD_VALUE = 0.3;

		//===================================
		//	Interface
		//===================================
		virtual void onInit();

		~WandererNodelet();

	private:
		//===================================
		//	Member Functions
		//===================================
		
		/**
		* populates the parameters from the launch parameters
		*/
		void fillConfigMap();

		/**
		* process the pointcloud and send command velocity messages
		*/
		void process(const pcl::PointCloud<pcl::PointXYZ>*  cloud);

		/**
		* returns a point in space to track based on the pointcloud
		*/
		pcl::PointXYZ getCentroidFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>*  cloud);

		/**
		* returns true if the function counted 'mMinBlobSize' points in the given range. if free space is true, the X range is being ignored.  
		*/
		bool enoughPointsDetected(const pcl::PointCloud<pcl::PointXYZ>*  cloud, float targetZ, bool freespace);

		/**
		* Calc the direction to turn
		*/
		int calcDirection(const pcl::PointCloud<pcl::PointXYZ>*  cloud, float targetZ);

		/**
		* Publishes a goal at the given 3D point for Rviz
		*/
		void publishGoal(double x, double y, double z);

		//***********************************
		// Callback functions
		//***********************************

		/**
		* Receives PointCloud msg (assumed from the designated camera to be used for tracking) and
		*	passes the data to the follower function
		*/
		void pointcloudCameraCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
		
		//***********************************
		// Debug/Visualize functions
		//***********************************

		/** 
		* Publishes a marker at the given 3D point for Rviz
		*/
		void publishMarker(double x,double y,double z);

		//***********************************
		// Utils functions
		//***********************************

		/**
		* simple str->bool
		*/
		bool to_bool(std::string str);

		long msecTime();

		//===================================
		// Dynamic reconfigure
		//===================================

		/**
		* Dynamic Reconfigure Callback
		*/
		void ConfigureCallback(realsense_wanderer::wandererConfig &config, uint32_t level);

		//===================================
		//	Member Variables
		//===================================
		
		ros::Subscriber mSub; /**< subscriber to the pointcloud topic */
		ros::Publisher mMarkerPub; /**< publisher of the marker message  */
		ros::Publisher mGoalPub; /**< publisher of the goal (position of the target) in 3D  */

		std::map<std::string,std::string> mConfig; /**< holds all the configuration received from the command line */

		double mMinY; /**< The minimum y position of the points in the box. */
		double mMaxY; /**< The maximum y position of the points in the box. */
		double mMinX; /**< The minimum x position of the points in the box. */
		double mMaxX; /**< The maximum x position of the points in the box. */
		double mMinZ; /**< The minimum z position of the points in the box. */
		double mMaxZ; /**< The maximum z position of the points in the box. */
		int    mMinBlobSize; /**< Minimum blob size parameter */
		bool   mEnabled; /**< Enable/disable flag */

		std::string mFrame; /**< Frame ID for the Marker message */


		boost::shared_ptr<dynamic_reconfigure::Server<realsense_wanderer::wandererConfig> > mReconfigureServer; /**< Reconfigure server */

		//Flags for goal setting
		bool mRotating; /**< Flag to keep if the goal is set currently to rotate */
		double mDirection;/**< Flag to keep the last direction of the rotation */

	};
}


#endif // WANDERER_NODELET

