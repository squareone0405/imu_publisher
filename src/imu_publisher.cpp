#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <RTIMULib.h>

using namespace std;

class ImuPublisher{
public:
    ImuPublisher(ros::NodeHandle &n){
        imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
		std::cout<<"init"<<std::endl;
        imu_settings = new RTIMUSettings(SETTINGS_FILE, true);
        imu = RTIMU::createIMU(imu_settings);
        imu->IMUInit();
        std::cout<<"IMU Name: " << imu->IMUName()<<std::endl;
        std::cout<<"init done"<<std::endl;
    }
    
    void start(){
        while(ros::ok()){
            process();
            ros::spinOnce();
        }
    }

    void printData(RTIMU_DATA data){
        cout<<"timestamp:\t"<<data.timestamp<<endl;
        cout<<"fusionPoseValid:\t"<<data.fusionPoseValid<<endl;
        if(data.fusionPoseValid)
            cout<<data.fusionPose.x() << "\t" << data.fusionPose.y() << "\t" << data.fusionPose.z() <<endl;
        cout<<"fusionQPoseValid:\t"<<data.fusionQPoseValid<<endl;
        if(data.fusionQPoseValid)
            cout<<data.fusionQPose.scalar() << "\t" << data.fusionQPose.x() << "\t" << data.fusionQPose.y() 
            << "\t" << data.fusionQPose.z() <<endl;
        cout<<"gyroValid:\t"<<data.gyroValid<<endl;
        if(data.gyroValid)
            cout<<data.gyro.x() << "\t" << data.gyro.y() << "\t" << data.gyro.z() <<endl;
        cout<<"accelValid:\t"<<data.accelValid<<endl;
        if(data.accelValid)
            cout<<data.accel.x() << "\t" << data.accel.y() << "\t" << data.accel.z() <<endl;
        cout<<"compassValid:\t"<<data.compassValid<<endl;
        if(data.compassValid)
            cout<<data.compass.x() << "\t" << data.compass.y() << "\t" << data.compass.z() <<endl;
    }

    void process(){
        while (imu->IMURead()) {
            RTIMU_DATA data = imu->getIMUData();
			sensor_msgs::Imu imu_msg;
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.header.frame_id = string("imu_link");
			for (int i = 0; i < 9; i++) {
				imu_msg.orientation_covariance[i] = 0;
				imu_msg.angular_velocity_covariance[i] = 0;
				imu_msg.linear_acceleration_covariance[i] = 0;
			}
            if(data.fusionQPoseValid){
                imu_msg.orientation.w = data.fusionQPose.scalar();
                imu_msg.orientation.x = data.fusionQPose.x();
                imu_msg.orientation.y = data.fusionQPose.y();
                imu_msg.orientation.z = data.fusionQPose.z();
            }
            else continue;
			if(data.gyroValid){
				imu_msg.angular_velocity.x = data.gyro.x();
				imu_msg.angular_velocity.y = data.gyro.y();
				imu_msg.angular_velocity.z = data.gyro.z();
			}
			else continue;
			if(data.accelValid){
				imu_msg.linear_acceleration.x = data.accel.x();
			    imu_msg.linear_acceleration.y = data.accel.y();
			    imu_msg.linear_acceleration.z = data.accel.z();
			}
            else continue;
			imu_pub.publish(imu_msg);
			//printData(data);
        }
    }
    
private:
    //const char* SETTINGS_FILE = "/home/nvidia/imu_ws/9250.ini";
	const char* SETTINGS_FILE = "/home/squareone/imu_ws/RTIMULib.ini";
    ros::Publisher imu_pub;
    RTIMU* imu;
    RTIMUSettings* imu_settings;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle n;
    ImuPublisher imu_pub = ImuPublisher(n);
    imu_pub.start();
    return 0;
}
