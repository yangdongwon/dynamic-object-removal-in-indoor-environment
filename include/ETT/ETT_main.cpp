// Standard libraries
#include<iostream>
#include <vector>
#include <cmath>
#include <math.h>
#include <string>

// Matrix calculation library
#include <Eigen/Eigen>
#include "ETT.h"

// For ROS(Robot operating system)
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
//#include "ETTv1/ETT.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>

// For PCL
#include <pcl_conversions/pcl_conversions.h> // ROS와 PCL(Point Cloud Library) 간의 변환을 위한 헤더 파일
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

// Declaration name space
using namespace std;
using namespace Eigen;

MatrixXd z;
vector<double> mlambda = {20};
vector<double> Idx = {};
vector<double> xv = {0,0,0,0,0,0};
vector<MatrixXd> xhat, Shape, Phat, Pshape;
Parameters param;

class Node {
public:
    Node(){
		/*
        pub_ = nh_.advertise<ETTv1::ETT>("my_topic", 10);
        sub_ = nh_.subscribe("/contour_cloud", 10, &Node::callback, this);
        sub_test = nh_.subscribe("/Dof", 10, &Node::callback2, this);
        pub_test = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
        pub_vehicle = nh_.advertise<visualization_msgs::Marker>("visualization_marker",10);
		*/
        pub_target = nh_.advertise<visualization_msgs::Marker>("Target_with_ellipse", 10);
        pub_DynPts = nh_.advertise<sensor_msgs::PointCloud2>("DynamicPoint", 10);

    Parameters params;
    // Initialize parameters.
    params.initialize_ETT_param(param);

    MatrixXd x1 = MatrixXd::Ones(6,1);
    x1 << -3.1, -0, 0.4, 0, 0, 0;
        MatrixXd P1 = param.Phat;
    P1 = param.Phat;
    MatrixXd P2 = param.Phat;
    P2 = param.Phat;

    MatrixXd S1 = MatrixXd::Ones(5,1);
    S1 << 0.0, 0.0, 0.3, 0.3, 1.0;

    MatrixXd p1 = param.Pshape;
    p1 = param.Pshape;
    MatrixXd p2 = param.Pshape;
    p2 = param.Pshape;

    xhat.push_back(x1);
    Shape.push_back(S1);
    Phat.push_back(P1);
    Pshape.push_back(p1);
   }
    void update(){
        //ETTv1::ETT msg;
        ETTJPDA ETT;
        // ROS_INFO("Pub");
        ETT.ETT_predict(xhat,Phat,Shape,Pshape,param);
        // ROS_INFO("Predict");
        mlambda = ETT.Gen_mlanbda(Shape,param);
        // ROS_INFO("Gen mlambda");
        MatrixXd beta = ETT.ETJPDA(xhat,Phat,Shape,Pshape,z,mlambda,param);
        // ROS_INFO("ETJPDA");
        ETT.ETT_update(xhat,Phat,Shape,Pshape,z,beta,param);
        // ROS_INFO("ETT_update");
        //ETT.Registration(xhat,Phat,Shape,Pshape,z,param);
        z = MatrixXd::Zero(3,0);
        //cout << "V: " << xhat[0] << endl;// << ", " << Shape[0](3,0) << endl;
        drawEllipse(xhat,Shape);
        //pub_.publish(msg);
    }

	void get(std_msgs::Header header, pcl::PointCloud<pcl::PointXYZI>& contourCloud, std::vector<int>& clusterIdx){
        ros::Time timestamp = header.stamp;
        //ROS_INFO("PointC loud2 timestamp: %f", timestamp.toSec());
        //cout << "What" << timestamp.toSec() << endl;
        int sec = timestamp.sec;
        int nsec = timestamp.nsec;
        cout << sec << ", " << nsec << endl;
		if(sec == 1713180173 && nsec == 931666127)
        {
            double nx_ = 25.5;
            double ny_ = 1.3;
            cout << "Get!!!" << endl;
            MatrixXd x1 = MatrixXd::Ones(6,1);
            x1 << nx_, ny_, 0.4, 0, 0, 0;
            MatrixXd S1 = MatrixXd::Ones(5,1);
            S1 << 0.0, 0.0, 0.3, 0.3, 1.0;
            MatrixXd P1 = MatrixXd::Zero(6,6);
            P1 << 0.3, 0, 0, 0, 0, 0,
                  0, 0.3, 0, 0, 0, 0,
                  0, 0, 0.1, 0, 0, 0,
                  0, 0, 0, 0.1, 0, 0,
                  0, 0, 0, 0, 0.1, 0,
                  0, 0, 0, 0, 0, 0.1;

            MatrixXd p1 = MatrixXd::Zero(5,5);
            p1 << 0.001*3.14/180, 0, 0, 0, 0,
                  0, 0.001*3.14/180, 0, 0, 0,
                  0, 0, 0.005, 0, 0,
                  0, 0, 0, 0.005, 0,
                  0, 0, 0, 0, 0.005;

            xhat.push_back(x1);
            Phat.push_back(P1);
            Shape.push_back(S1);
            Pshape.push_back(p1);
        }
        if(sec == 1713180184 && nsec == 31846195)
        {
            double nx_ = 38;
            double ny_ = 0;
            cout << "Get!!!" << endl;
            MatrixXd x1 = MatrixXd::Ones(6,1);
            x1 << nx_, ny_, 0.4, 0, 0, 0;
            MatrixXd S1 = MatrixXd::Ones(5,1);
            S1 << 0.0, 0.0, 0.3, 0.3, 1.0;
            MatrixXd P1 = MatrixXd::Zero(6,6);
            P1 << 0.1, 0, 0, 0, 0, 0,
                  0, 0.1, 0, 0, 0, 0,
                  0, 0, 0.1, 0, 0, 0,
                  0, 0, 0, 0.1, 0, 0,
                  0, 0, 0, 0, 0.1, 0,
                  0, 0, 0, 0, 0, 0.1;

            MatrixXd p1 = MatrixXd::Zero(5,5);
            p1 << 0.001*3.14/180, 0, 0, 0, 0,
                  0, 0.001*3.14/180, 0, 0, 0,
                  0, 0, 0.005, 0, 0,
                  0, 0, 0, 0.005, 0,
                  0, 0, 0, 0, 0.005;

            xhat.push_back(x1);
            Phat.push_back(P1);
            Shape.push_back(S1);
            Pshape.push_back(p1);
        }

        pcl::PointCloud<pcl::PointXYZI> cloud = contourCloud;
        int point_count = cloud.size();

        // Extract the intensity information (cluster ID)
        MatrixXd z_temp = MatrixXd::Zero(3, cloud.size());
        vector<double> Idx_temp = {};
        for(int i = 0; i < point_count; i++)
        {
            z_temp(0,i) = cloud.points[i].x;
            z_temp(1,i) = cloud.points[i].y;
            z_temp(2,i) = cloud.points[i].z;
            Idx_temp.push_back(cloud.points[i].intensity);
        }
        z = z_temp;
        Idx = Idx_temp;
        // Create KD tree
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
        kd_tree->setInputCloud(cloud.makeShared());

        // Define the search point
        //pcl::PointXYZI search_point(xv[0], xv[1], xv[2]);
        //pcl::PointXYZI search_point(static_cast<float>(xv[0]), static_cast<float>(xv[1]), static_cast<float>(xv[2]));
        pcl::PointXYZI search_point;
        search_point.x = static_cast<float>(xv[0]);
        search_point.y = static_cast<float>(xv[1]);
        search_point.z = static_cast<float>(xv[2]);
        
        // Search for points within 5.0m radius from the search point
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        kd_tree->radiusSearch(search_point, 5.0, point_indices, point_distances);

        // Extract the points within the radius
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; i < point_indices.size(); ++i) {
            filtered_cloud->push_back(cloud.points[point_indices[i]]);
        }
        // filtered_cloud에서 filtered_points로 변환
        vector<vector<double>> filtered_points;

        for(size_t i = 0; i < filtered_cloud->size(); ++i)
        {
            pcl::PointXYZI point = filtered_cloud->points[i];
            std::vector<double> point_data = {point.x, point.y, point.z, point.intensity};
            filtered_points.push_back(point_data);
        }
        // registration
        // calculateClusterMeanAndVariance(filtered_points);

		/*
        // Convert filtered point cloud back to PointCloud2
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*filtered_cloud, output_cloud);
        output_cloud.header = msg->header;

        // Publish the filtered point cloud
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        pub.publish(output_cloud);
		*/

        // Estimated dynamic pts
        pcl::PointCloud<pcl::PointXYZI>::Ptr DynPts(new pcl::PointCloud<pcl::PointXYZI>);
        for(int k = 0; k < xhat.size(); k++)
        {
            // cout << "My" << endl;
            double w = 0.1;
            double xk = xhat[k](0,0);
            double yk = xhat[k](1,0);
            double zk = xhat[k](2,0);
            double psik = Shape[k](0,0);
            double pitchk = Shape[k](1,0);
            double rollk = 0;
            double l1k = Shape[k](2,0);
            double l2k = Shape[k](3,0);
            double l3k = Shape[k](4,0);
            // cout << "My" << endl;
            double V = sqrt(pow(xhat[k](3,0),2)+pow(xhat[k](4,0),2));

            if(V > 0.2)
            {
                for (int i = 0; i < cloud.size(); i++)
                {
                    double px = cloud.points[i].x;
                    double py = cloud.points[i].y;
                    double pz = cloud.points[i].z;
                    double dx = px - xk;
                    double dy = py - yk;
                    double dz = pz - zk;
                    // double A1 = (dx*cos(psik)-dy*sin(psik))*cos(pitchk)-dz*sin(pitchk);
                    // double A2 = (dx*sin(psik)+dy*sin(psik))*cos(rollk)-(dz*cos(pitchk)+(dx*cos(psik)-dy*sin(psik))*sin(pitchk))*sin(rollk);
                    // double A3 = (dx*sin(psik)+dy*cos(psik))*sin(rollk)+(dz*cos(pitchk)+(dx*cos(psik)-dy*sin(psik))*sin(pitchk))*cos(rollk);
                    double A1 = (dx*cos(psik)-dy*sin(psik))*cos(pitchk)-dz*sin(pitchk);
                    double A2 = (dx*sin(psik)+dy*cos(psik))*cos(rollk)-(dz*cos(pitchk)+(dx*cos(psik)-dy*sin(psik))*sin(pitchk))*sin(rollk);
                    double A3 = (dx*sin(psik)+dy*cos(psik))*sin(rollk)+(dz*cos(pitchk)+(dx*cos(psik)-dy*sin(psik))*sin(pitchk))*cos(rollk);
                    // double value2 = 

                    double value = (A1/(l1k+w*l1k))*(A1/(l1k+w*l1k))+(A2/(l2k+w*l2k))*(A2/(l2k+w*l2k))+(A3/(l3k+w*l3k))*(A3/(l3k+w*l3k));
                    if(value < 1)
                    {
                        DynPts->push_back(cloud.points[i]);
						clusterIdx.push_back(cloud.points[i].intensity);
                    }
                }
            }
        }
        //pcl::fromPCLPointCloud2(pcl_pc, cloud);
        sensor_msgs::PointCloud2 output_cloud2;
        pcl::toROSMsg(*DynPts, output_cloud2);
        output_cloud2.header = header;
        pub_DynPts.publish(output_cloud2);
    }

	void drawEllipse(vector<MatrixXd> Target_arr, vector<MatrixXd> Shape_arr)
    {
        int Nt = Target_arr.size();
        vector<visualization_msgs::Marker> markers(Nt);
        //cout << "Nt: " << Nt << endl;
        for(int i = 0; i < Nt; i++)
        {
            double xi = Target_arr[i](0,0);
            double yi = Target_arr[i](1,0);
            double zi = Target_arr[i](2,0);
            double pi = 0;
            double qi = Shape_arr[i](1,0);
            double ri = Shape_arr[i](0,0);
            double l1 = Shape_arr[i](2,0);
            double l2 = Shape_arr[i](3,0);
            double l3 = Shape_arr[i](4,0);
            markers[i].header.frame_id = "os_sensor";
            markers[i].header.stamp = ros::Time::now();
            markers[i].ns = "ellipse";
            markers[i].id = i;
            markers[i].type = visualization_msgs::Marker::SPHERE;
            markers[i].action = visualization_msgs::Marker::ADD;

            // set position
            markers[i].pose.position.x = xi;
            markers[i].pose.position.y = yi;
            markers[i].pose.position.z = zi;
            // set orientation
            tf2::Quaternion quat;
            quat.setRPY(pi,qi,ri);
            markers[i].pose.orientation.x = quat.x();
            markers[i].pose.orientation.y = quat.y();
            markers[i].pose.orientation.z = quat.z();
            markers[i].pose.orientation.w = quat.w();    

            // set scale (size of ellipse)
            markers[i].scale.x = l1+0.3*l1; // Set semi-major axis along x
            markers[i].scale.y = l2+0.3*l2; // Set semi-minor axis along y
            markers[i].scale.z = l3+0.3*l3; // Set semi-minor axis along z        
            // set color (blue)
            markers[i].color.a = 1.0; // Alpha (opacity)
            markers[i].color.r = 0.0; // Red
            markers[i].color.g = 0.0; // Green
            markers[i].color.b = 1.0; // Blue

            pub_target.publish(markers[i]);
        }
    }
    void calculateClusterMeanAndVariance(const std::vector<std::vector<double>> filtered_points)
    {
        int max_idx = 0;
        for(int i = 0; i < filtered_points.size(); i++)
        {
            if(max_idx < filtered_points[i][3])
            {
                max_idx = filtered_points[i][3];
            }
        }
        if(max_idx != -10)
        {
            vector<vector<double>> PCD(max_idx,vector<double> (7,0));
            // [idx][:]
            // 0: cnt
            // 1: mean_x
            // 2: mean_y
            // 3: mean_z
            // 2: var_y
            // 3: 
            // 
            for(int i = 0; i < filtered_points.size(); i++)
            {
                if(filtered_points[i][3] != -10)
                {
                    int idx = filtered_points[i][3] - 1;
                    int n = PCD[idx][0];
                    double mx = PCD[idx][1];
                    double my = PCD[idx][2];
                    double mz = PCD[idx][3];
                    double vx = PCD[idx][4];
                    double vy = PCD[idx][5];
                    double vz = PCD[idx][6];
                    double new_x = filtered_points[i][0];
                    double new_y = filtered_points[i][1];
                    double new_z = filtered_points[i][2];

                    double new_mx = ((mx*n)+new_x)/(n+1);
                    double new_my = ((my*n)+new_y)/(n+1);
                    double new_mz = ((mz*n)+new_z)/(n+1);
                    double new_vx = ((vx*n)+(new_x-new_mx))/(n+1);
                    double new_vy = ((vy*n)+(new_y-new_my))/(n+1);
                    double new_vz = ((vz*n)+(new_z-new_mz))/(n+1);
                    
                    PCD[idx][0] = PCD[idx][0] + 1;
                    PCD[idx][1] = new_mx;
                    PCD[idx][2] = new_my;
                    PCD[idx][3] = new_mz;
                    PCD[idx][4] = new_vx;
                    PCD[idx][5] = new_vy;
                    PCD[idx][6] = new_vz;
                    
                }
            }
            
            for(int i = 0; i < max_idx; i++)
            {
                // PCD[i]
                // cout << PCD[i][0] << ", " << PCD[i][1] <<  ", " << PCD[i][2] << ", " << PCD[i][3] << ", " << PCD[i][4] << ", " << PCD[i][5] << ", " << PCD[i][6]  << endl;
                if(PCD[i][0] > 25 && PCD[i][1] > 0.3 )
                {
                    double nx = PCD[i][1];
                    double ny = PCD[i][2];
                    double nz = PCD[i][3];
                    double nlx = PCD[i][4];
                    double nly = PCD[i][5];
                    double nlz = PCD[i][6];
                    double nalpha = 0.0001;
                    double nbeta = 0.0001;
                    
                    int nt = xhat.size();
                    MatrixXd NTarget = MatrixXd::Zero(6,1);
                    NTarget << nx, ny, nz, 0, 0, 0;
                    MatrixXd NShape = MatrixXd::Zero(5,1);
                    NShape << nalpha, nbeta, nlx, nly, nlz;
                    MatrixXd NPhat = MatrixXd::Zero(6,6);
                    NPhat << 0.01, 0, 0, 0, 0, 0,
                             0, 0.01, 0, 0, 0, 0,
                             0, 0, 0.01, 0, 0, 0,
                             0, 0, 0, 0.01, 0, 0,
                             0, 0, 0, 0, 0.01, 0,
                             0, 0, 0, 0, 0, 0.01;
                    MatrixXd NPshape = MatrixXd::Zero(5,5);
                    NPshape << 0.0001, 0, 0, 0, 0,
                               0, 0.0001, 0, 0, 0,
                               0, 0, 0.00001, 0, 0,
                               0, 0, 0, 0.00001, 0,
                               0, 0, 0, 0, 0.00001;
                    xhat.push_back(NTarget);
                    Phat.push_back(NPhat);
                    Shape.push_back(NShape);
                    Pshape.push_back(NPshape);
                }
            }
        }
//        cout << "Test run" << endl;
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub_test;
    ros::Publisher pub_test;
    ros::Publisher pub_vehicle;
    ros::Publisher pub_target;
    ros::Publisher pub_DynPts;
};

/*
int main(int argc, char **argv){
    ros::init(argc, argv, "my_node");
    Node node;
    ros::Rate loop_rate(10);
    while(ros::ok()){
        node.ETT_publisher(xhat,Shape,Phat,Pshape,param);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
*/
