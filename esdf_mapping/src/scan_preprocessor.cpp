#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ScanPreprocessor
{
public:
    ScanPreprocessor()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        std::string input_topic;
        std::string output_topic;

        pnh.param<std::string>("input_scan_topic", input_topic, "front/scan");
        pnh.param<std::string>("output_scan_topic", output_topic, "front/scan_preprocessed");
        pnh.param("replace_inf_range", replace_inf_range_, 100.0f);

        scan_pub_ = nh.advertise<sensor_msgs::LaserScan>(output_topic, 1);
        scan_sub_ = nh.subscribe(input_topic, 1, &ScanPreprocessor::scanCallback, this);
    }

private:
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
    {
        output_msg_.header = scan_msg->header;
        output_msg_.angle_min = scan_msg->angle_min;
        output_msg_.angle_max = scan_msg->angle_max;
        output_msg_.angle_increment = scan_msg->angle_increment;
        output_msg_.time_increment = scan_msg->time_increment;
        output_msg_.scan_time = scan_msg->scan_time;
        output_msg_.range_min = scan_msg->range_min;
        output_msg_.range_max = scan_msg->range_max;

        const std::size_t range_size = scan_msg->ranges.size();
        output_msg_.ranges.resize(range_size);
        for (std::size_t i = 0; i < range_size; ++i)
        {
            const float range = scan_msg->ranges[i];
            output_msg_.ranges[i] = std::isinf(range) ? replace_inf_range_ : range;
        }

        output_msg_.intensities = scan_msg->intensities;
        scan_pub_.publish(output_msg_);
    }

    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;
    sensor_msgs::LaserScan output_msg_;
    float replace_inf_range_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_preprocessor_node");
    ScanPreprocessor node;
    ros::spin();
    return 0;
}
