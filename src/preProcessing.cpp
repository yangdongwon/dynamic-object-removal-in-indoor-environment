#include "preProcessing.hpp"

PreProcessing::PreProcessing()
{
	subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 5, &PreProcessing::pointCloudHandler, this, ros::TransportHints().tcpNoDelay());

	pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("extracted_cloud", 1);
	pubInformation = nh.advertise<dynamic::information>("information", 1);

	extractedCloud.reset(new pcl::PointCloud<PointType>());

	velodyneCloud.reset(new pcl::PointCloud<VelodynePointType>());
	ousterCloud.reset(new pcl::PointCloud<OusterPointType>());
	livoxCloud.reset(new pcl::PointCloud<LivoxPointType>());

	info.point_range_2D.assign(channel*resolution, 0);
}

PreProcessing::~PreProcessing(){}

void PreProcessing::resetParam()
{
	extractedCloud->clear();
}

void PreProcessing::pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
	resetParam();

	if(!convertCloudFormat(pointCloudMsg))
		return;

	if(!checkValidCloud())
		return;

	saveCloudInformation();

	publishCloudInfo();
}

bool PreProcessing::convertCloudFormat(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
	cloudDataQueue.push_back(*pointCloudMsg);
	if(cloudDataQueue.size() >= 10){
		std::cout << "Preprocessing is too slow!\n";
		return false;
	}

	cloudMsg = std::move(cloudDataQueue.front());
	info.origin_cloud = cloudMsg;
	cloudDataQueue.pop_front();
	cloudHeader = cloudMsg.header;

	if(sensor == Sensor::VELODYNE){
		pcl::moveFromROSMsg(cloudMsg, *velodyneCloud);
		extractedCloud->resize(velodyneCloud->points.size());
		convertCloud(velodyneCloud, extractedCloud);
	}
    else if(sensor == Sensor::OUSTER){
		pcl::moveFromROSMsg(cloudMsg, *ousterCloud);
		extractedCloud->resize(ousterCloud->points.size());
		convertCloud(ousterCloud, extractedCloud);
	}
    else if(sensor == Sensor::LIVOX){
		pcl::moveFromROSMsg(cloudMsg, *livoxCloud);
		extractedCloud->resize(livoxCloud->points.size());
		convertCloud(livoxCloud, extractedCloud);
	}

	return true;
}

bool PreProcessing::checkValidCloud()
{
	if(!extractedCloud->is_dense){
		printf("no dense!\n");
		return false;
	}

	return true;
}

void PreProcessing::saveCloudInformation()
{
	int size = extractedCloud->points.size();
	for(int i = 0; i < size; i++)
		info.point_range_2D[i] = distance2D(extractedCloud->points[i]);

	info.header = cloudHeader;
	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg(*extractedCloud, cloud);
	info.extract_cloud = cloud;
}

void PreProcessing::publishCloudInfo()
{
	pubInformation.publish(info);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamic");

	PreProcessing PP;

	ROS_INFO("\033[1;32m----> PreProcessing Started.\033[0m");

	ros::spin();

	return 0;
}
