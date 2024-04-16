#include "dynamicFeature.hpp" 

DynamicFeature::DynamicFeature()
{
	subInformation = nh.subscribe<dynamic::information>("information", 1, &DynamicFeature::informationHandler, this, ros::TransportHints().tcpNoDelay());

	subGeneratedOdometry = nh.subscribe<nav_msgs::Odometry>(generatedOdomTopic, 10, &DynamicFeature::generatedOdometryHandler, this, ros::TransportHints().tcpNoDelay());
	subGeneratedImu = nh.subscribe<nav_msgs::Odometry>("/odometry/imu_incremental", 100, &DynamicFeature::generatedImuHandler, this, ros::TransportHints().tcpNoDelay());

	pubStaticPoints = nh.advertise<sensor_msgs::PointCloud2>("static_points", 1);
	pubDynamicPoints = nh.advertise<sensor_msgs::PointCloud2>("dynamic_points", 1);
        
	pubTempPointCloud = nh.advertise<sensor_msgs::PointCloud2>("contour_cloud", 1);
	pubPrePointCloud = nh.advertise<sensor_msgs::PointCloud2>("pre_cloud", 1);
	pubPostPointCloud = nh.advertise<sensor_msgs::PointCloud2>("post_cloud", 1);

	velodyneCloud.reset(new pcl::PointCloud<VelodynePointType>());
	ousterCloud.reset(new pcl::PointCloud<OusterPointType>());
	livoxCloud.reset(new pcl::PointCloud<LivoxPointType>());

	ds = new Dbscan(minPoints, epslion);
	ETT = new Node();
	extractedCloud.reset(new pcl::PointCloud<PointType>());
	predictedDynamicPoints.reset(new pcl::PointCloud<PointType>());
	contourCloud.reset(new pcl::PointCloud<PointType>());

	dynamicPointArray = new bool[channel*resolution];

	isOdomGenerated = false;
	isFrameStackUp = false;
}

DynamicFeature::~DynamicFeature(){}

void DynamicFeature::resetParam()
{
	predictedDynamicPoints->clear();
	contourCloud->clear();

	idxContainer.clear();
	tempClusterContainer.clear();
	clusterContainer.clear();
	clusterIdx.clear();

	clusterNum = 0;
}

void DynamicFeature::informationHandler(const dynamic::information::ConstPtr& infoMsg)
{
	static int pre = 1;
	static float max_process_time = 0;
	static float pre_process_time = 0;

	info = *infoMsg;
	cloudHeader = infoMsg->header;
	pcl::fromROSMsg(infoMsg->extract_cloud, *extractedCloud);

	if(sensor == Sensor::OUSTER){
		pcl::PointCloud<OusterPointType>::Ptr tempCloud(new pcl::PointCloud<OusterPointType>);
		pcl::fromROSMsg(info.origin_cloud, *tempCloud);
		ousterCloudQueue.push_back(*tempCloud);
	}

	// utility
	float process_time = timeCheck(std::bind(&DynamicFeature::findPoints, this));

	max_process_time = std::max(max_process_time, process_time);
	printf("max process time: %f\n", max_process_time);

	float a = float(pre - 1) / pre;
	pre++;
	float avg_time = a * pre_process_time + (1 - a) * process_time;
	pre_process_time = avg_time;
	printf("avg process time: %f\n\n", avg_time);
}

// lio-sam odometry
void DynamicFeature::generatedImuHandler(const nav_msgs::Odometry::ConstPtr& odomImuMsg)
{
	std::lock_guard<std::mutex> lock(imuMtx);
	generatedImuDataQueue.push_back(*odomImuMsg);
}

// lio-sam odometry
void DynamicFeature::generatedOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
	isOdomGenerated = true;
	std::lock_guard<std::mutex> lock(odomMtx);
	generatedOdomDataQueue.push_back(*odomMsg);
}

void DynamicFeature::findPoints()
{
	std::lock_guard<std::mutex> lock(mtx);

	resetParam();

	extractContour();

	clustering2D();

	divideCluster();

	revertCluster();

	saveFrame();

	if(isFrameStackUp){
		syncLidarOdom();

		extendedTargetTracking();

		findUnmatchedPoints();
	}

	extractStaticPoints();

	publishPointCloud();
}

void DynamicFeature::extractContour()
{
	bool isStart = false, isEnd = false, isCorrectSeq;
	int startIdx, endIdx;

	for (int i = 1; i < extractedCloud->points.size()-1; i++){
		float dist1 = info.point_range_2D[i - 1];
		float dist2 = info.point_range_2D[i];
		float dist3 = info.point_range_2D[i + 1];

		bool isFarLeftFront = (dist2 - dist1) > distDiffThreshold;
		bool isFarLeftBack = (dist1 - dist2) > distDiffThreshold;
		bool isFarRightFront = (dist2 - dist3) > distDiffThreshold;
		bool isFarRightBack = (dist3 - dist2) > distDiffThreshold;

		if((isFarLeftFront || isFarLeftBack) && (isFarRightFront || isFarRightBack))
			continue;

		if(dist1 == 0 || dist2 == 0 || dist3 == 0)
			continue;

		if(dist2 < distThreshold){
			if(isFarLeftFront){
				startIdx = i;
				isStart = true;
				isCorrectSeq = false;
			}
			else if(isFarLeftBack){
				startIdx = i;
				isStart = true;
				isCorrectSeq = false;
			}
			else if(isFarRightFront){
				endIdx = i;
				isEnd = true;
				isCorrectSeq = true;
			}
			else if(isFarRightBack){
				endIdx = i;
				isEnd = true;
				isCorrectSeq = true;
			}
		}

		if(isStart && isEnd && isCorrectSeq){
			isStart = false;
			isEnd = false;

			float cDistance = 0;
			for(int j = startIdx; j < endIdx; j++){
				cDistance += pointDistance2D(extractedCloud->points[j], 
											 extractedCloud->points[j+1]);
			}
			float uDistance = pointDistance2D(extractedCloud->points[startIdx], extractedCloud->points[endIdx]);	
			
			if(cDistance / uDistance > 3.0)
				continue;

			if(pointAngle2D(extractedCloud->points[startIdx], extractedCloud->points[endIdx]) > 160)
				continue;

			if(endIdx - startIdx > 2){
				PointType point;
				point.x = (extractedCloud->points[startIdx].x + extractedCloud->points[endIdx].x)/2;
				point.y = (extractedCloud->points[startIdx].y + extractedCloud->points[endIdx].y)/2;
				point.z = (extractedCloud->points[startIdx].z + extractedCloud->points[endIdx].z)/2;

				predictedDynamicPoints->emplace_back(point);
				idxContainer.emplace_back(std::make_pair(startIdx, endIdx));
			}
		}
	}

	info.point_range_2D.clear();
}

void DynamicFeature::clustering2D()
{
	int pointSize = predictedDynamicPoints->points.size();

	vector<Point> points;
	Point *p = new Point[pointSize];
	for(int i = 0; i < pointSize; i++){
		p[i].clusterID = UNCLASSIFIED;
		p[i].x = predictedDynamicPoints->points[i].x;
		p[i].y = predictedDynamicPoints->points[i].y;
		points.push_back(p[i]);
	}
	delete p;

	ds->dbscan(points);

	for(int i = 0; i < pointSize; i++){
		int clusterID = points[i].clusterID;
		clusterNum = std::max(clusterNum, clusterID);

		if(clusterID > 0){
			predictedDynamicPoints->points[i].intensity = clusterID;

			IntervalPoint itv;
			itv.midPoint = predictedDynamicPoints->points[i];
			itv.startIdx = idxContainer[i].first;
			itv.endIdx = idxContainer[i].second;

			tempClusterContainer.emplace(clusterID, itv);
		}
		else
			predictedDynamicPoints->points[i].intensity = -10;
	}
}

void DynamicFeature::divideCluster()
{
	int newClusterID = 0;
	for(int i = 1; i <= clusterNum; i++){
		auto rangeIter = tempClusterContainer.equal_range(i);

		std::vector<IntervalPoint> tempClusterCloud;
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			if(iter != rangeIter.first &&
			std::prev(iter)->second.midPoint.z - iter->second.midPoint.z > 0.2){
				newClusterID += 1;
				for (auto& itv : tempClusterCloud) {
					itv.midPoint.intensity = newClusterID;
					clusterContainer.emplace(newClusterID, itv);
				}
				tempClusterCloud.clear();
			}
			tempClusterCloud.push_back(iter->second);
		}

		newClusterID += 1;
		for (auto& itv : tempClusterCloud) {
			itv.midPoint.intensity = newClusterID;
			clusterContainer.emplace(newClusterID, itv);
		}
	}

	clusterNum = newClusterID;
}

void DynamicFeature::revertCluster()
{
	for(int i = 1; i <= clusterNum; i++){
		auto rangeIter = clusterContainer.equal_range(i);
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			PointType startPoint, endPoint;
			startPoint.x = extractedCloud->points[iter->second.startIdx].x;
			startPoint.y = extractedCloud->points[iter->second.startIdx].y;
			startPoint.z = extractedCloud->points[iter->second.startIdx].z;
			startPoint.intensity = iter->first;

			endPoint.x = extractedCloud->points[iter->second.endIdx].x;
			endPoint.y = extractedCloud->points[iter->second.endIdx].y;
			endPoint.z = extractedCloud->points[iter->second.endIdx].z;
			endPoint.intensity = iter->first;

			contourCloud->push_back(startPoint);
			contourCloud->push_back(endPoint);
		}
	}
}

void DynamicFeature::saveFrame()
{
	FrameData data;
	data.header = cloudHeader;
	data.cloudData = *contourCloud;

	frameDataQueue.push_back(data);
	clusterIdxQueue.push_back(clusterContainer);

	if(frameDataQueue.size() >= 7)
		isFrameStackUp = true;
}

void DynamicFeature::syncLidarOdom()
{
	if(!isOdomGenerated)
		return;

	nav_msgs::Odometry odomData = generatedOdomDataQueue.front();
	static double curr6DOF[6];

	double roll, pitch, yaw;
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(odomData.pose.pose.orientation, orientation);

	tf::Matrix3x3 m(orientation);
	m.getRPY(roll, pitch, yaw);

	FrameData frameData = std::move(frameDataQueue.front());

	// cout << frameData.header.stamp.toSec() - odomData.header.stamp.toSec() << "\n";

	if(frameData.header.stamp.toSec() == odomData.header.stamp.toSec()){
		generatedOdomDataQueue.pop_front();

		curr6DOF[3] = roll;
		curr6DOF[4] = pitch;
		curr6DOF[5] = yaw;
	
		curr6DOF[0] = odomData.pose.pose.position.x;
		curr6DOF[1] = odomData.pose.pose.position.y;
		curr6DOF[2] = odomData.pose.pose.position.z;

		std::cout << "synchronization" <<"\n";
		for(int i = 0; i < 6; i++)
			std::cout << curr6DOF[i] << " ";
		std::cout << "\n";
	}
	else{
		curr6DOF[3] += (roll - curr6DOF[3]) / 2;
		curr6DOF[4] += (pitch - curr6DOF[4]) / 2;
		curr6DOF[5] += (yaw - curr6DOF[5]) / 2;
	
		curr6DOF[0] += (odomData.pose.pose.position.x - curr6DOF[0]) / 2;
		curr6DOF[1] += (odomData.pose.pose.position.y - curr6DOF[1]) / 2;
		curr6DOF[2] += (odomData.pose.pose.position.z - curr6DOF[2]) / 2;

		std::cout << "interpolation" << "\n";

		for(int i = 0; i < 6; i++)
			std::cout << curr6DOF[i] << " ";
		std::cout << "\n";
	}

	pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
	cloudTransform(curr6DOF, m, frameData.cloudData, *pointCloud);
	publishCloud(pubTempPointCloud, pointCloud, frameData.header.stamp, refFrame);

	contourCloudData = std::make_pair(frameData.header, *pointCloud);

	frameDataQueue.pop_front();

	if(sensor == Sensor::OUSTER)
		ousterCloudQueue.pop_front();
}

void DynamicFeature::cloudTransform(double pose[], tf::Matrix3x3 m, pcl::PointCloud<PointType>& src, pcl::PointCloud<PointType>& dst)
{
	Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();

	Eigen::Matrix3d newRotMat;
	tf::matrixTFToEigen(m, newRotMat);

    transMat.block<3, 3>(0, 0) = newRotMat;
    transMat.block<3, 1>(0, 3) << pose[0], pose[1], pose[2];

	pcl::transformPointCloud(src, dst, transMat);
}	

void DynamicFeature::extendedTargetTracking()
{
	if(!isOdomGenerated)
		return;

	ETT->update();
	ETT->get(contourCloudData.first, contourCloudData.second, clusterIdx);

	sort(clusterIdx.begin(), clusterIdx.end());
	clusterIdx.erase(unique(clusterIdx.begin(), clusterIdx.end()), clusterIdx.end());
}

void DynamicFeature::findUnmatchedPoints()
{
	std::fill(dynamicPointArray, dynamicPointArray+channel*resolution, false);

	auto tempClusterContainer = std::move(clusterIdxQueue.front());
	clusterIdxQueue.pop_front();

	for(int& cID : clusterIdx){
		auto rangeIter = tempClusterContainer.equal_range(cID);
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			int startIdx = iter->second.startIdx;
			int endIdx = iter->second.endIdx;

			for(int i = startIdx - 1; i <= endIdx + 1; i++)
				dynamicPointArray[i] = true;
		}
	}
}

void DynamicFeature::extractStaticPoints()
{
	//cout << frameDataQueue.size() << "\n";
	//cout << ousterCloudQueue.size() << "\n";

	if(sensor == Sensor::VELODYNE){
		pcl::fromROSMsg(info.origin_cloud, *velodyneCloud);

		pcl::PointCloud<VelodynePointType> tempCloud;
		for(int i = 0; i < velodyneCloud->points.size(); i++){
			if(!dynamicPointArray[i])
				tempCloud.emplace_back(velodyneCloud->points[i]);
		}

		publishCloud(pubStaticPoints, &tempCloud, cloudHeader.stamp, refFrame);
	}
	else if(sensor == Sensor::OUSTER){
		*ousterCloud = ousterCloudQueue.front();

		pcl::PointCloud<OusterPointType>::Ptr staticCloud(new pcl::PointCloud<OusterPointType>);
		pcl::PointCloud<OusterPointType>::Ptr dynamicCloud(new pcl::PointCloud<OusterPointType>);
		for(int i = 0; i < ousterCloud->points.size(); i++){
			if(!dynamicPointArray[i])
				staticCloud->emplace_back(ousterCloud->points[i]);
			else
				dynamicCloud->emplace_back(ousterCloud->points[i]);
		}

		publishCloud(pubStaticPoints, staticCloud, cloudHeader.stamp, refFrame);
		publishCloud(pubDynamicPoints, dynamicCloud, cloudHeader.stamp, refFrame);

		// publish remove 
		pcl::PointCloud<OusterPointType>::Ptr tempCloud1(new pcl::PointCloud<OusterPointType>);
		pcl::PointCloud<OusterPointType>::Ptr tempCloud2(new pcl::PointCloud<OusterPointType>);
		for(int i = 0; i < ousterCloud->points.size(); i++){
			bool isP = ousterCloud->points[i].z > -0.3 && ousterCloud->points[i].z < 1;
			if(!dynamicPointArray[i] && isP)
				tempCloud2->emplace_back(ousterCloud->points[i]);
			if(isP)
				tempCloud1->emplace_back(ousterCloud->points[i]);
		}

		pcl::PointCloud<OusterPointType>::Ptr dsTempCloud2(new pcl::PointCloud<OusterPointType>);
		pcl::VoxelGrid<OusterPointType> dsCloud2;
		dsCloud2.setInputCloud(tempCloud2);
		dsCloud2.setLeafSize(0.1, 0.1, 0.1);
		dsCloud2.filter(*dsTempCloud2);

		pcl::PointCloud<OusterPointType>::Ptr dsTempCloud1(new pcl::PointCloud<OusterPointType>);
		pcl::VoxelGrid<OusterPointType> dsCloud1;
		dsCloud1.setInputCloud(tempCloud1);
		dsCloud1.setLeafSize(0.1, 0.1, 0.1);
		dsCloud1.filter(*dsTempCloud1);

		publishCloud(pubPrePointCloud, dsTempCloud1, cloudHeader.stamp, "base_link");
		publishCloud(pubPostPointCloud, dsTempCloud2, cloudHeader.stamp, "base_link");
	}
    else if(sensor == Sensor::LIVOX){
		pcl::fromROSMsg(info.origin_cloud, *livoxCloud);

		pcl::PointCloud<LivoxPointType> tempCloud;
		for(int i = 0; i < livoxCloud->points.size(); i++){
			if(!dynamicPointArray[i])
				tempCloud.emplace_back(livoxCloud->points[i]);
		}

		publishCloud(pubStaticPoints, &tempCloud, cloudHeader.stamp, refFrame);
	}
}

void DynamicFeature::publishPointCloud()
{
	//publishCloud(pubDescriptorPoints, descriptorPoints, cloudHeader.stamp, refFrame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic");

    DynamicFeature DF;

    ROS_INFO("\033[1;32m----> Dynamic Feature Started.\033[0m");
   
    ros::spin();

    return 0;
}
