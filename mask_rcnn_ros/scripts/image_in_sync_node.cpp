#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ImageSyncPublisher
{
public:
    ImageSyncPublisher(ros::NodeHandle nh)
    {
        std::string inspector;
        nh.getParam("inspector", inspector); // Get inspector parameter
        // Append inspector namespace to topics
        std::string depth_topic = "/" + inspector + "/camera/depth/image_raw";
        std::string rgb_topic = "/" + inspector + "/camera/color/image_raw";
        std::string depth_info_topic = "/" + inspector + "/camera/depth/camera_info";
        std::string rgb_info_topic = "/" + inspector + "/camera/color/camera_info";

        // Publishers
        depth_pub = nh.advertise<sensor_msgs::Image>(depth_topic, 10);
        rgb_pub = nh.advertise<sensor_msgs::Image>(rgb_topic, 10);
        depth_info_pub = nh.advertise<sensor_msgs::CameraInfo>(depth_info_topic, 10);
        rgb_info_pub = nh.advertise<sensor_msgs::CameraInfo>(rgb_info_topic, 10);

        // Subscribers
        depth_sub.subscribe(nh, depth_topic, 1);
        rgb_sub.subscribe(nh, rgb_topic, 1);
        depth_info_sub.subscribe(nh, depth_info_topic, 1);
        rgb_info_sub.subscribe(nh, rgb_info_topic, 1);

        // Synchronizer
        sync.reset(new Sync(SyncPolicy(10), depth_sub, rgb_sub, depth_info_sub, rgb_info_sub));
        sync->registerCallback(boost::bind(&ImageSyncPublisher::sync_callback, this, _1, _2, _3, _4));

        // Timer
        timer = nh.createTimer(ros::Duration(1.0/11.0), &ImageSyncPublisher::timer_callback, this);
    }

    void sync_callback(const sensor_msgs::ImageConstPtr& depth_msg,
                       const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::CameraInfoConstPtr& depth_info_msg,
                       const sensor_msgs::CameraInfoConstPtr& rgb_info_msg)
    {
        latest_depth_msg = depth_msg;
        latest_rgb_msg = rgb_msg;
        latest_depth_info_msg = depth_info_msg;
        latest_rgb_info_msg = rgb_info_msg;
    }

    void timer_callback(const ros::TimerEvent&)
    {
        if (latest_depth_msg && latest_rgb_msg && latest_depth_info_msg && latest_rgb_info_msg)
        {
            depth_pub.publish(latest_depth_msg);
            rgb_pub.publish(latest_rgb_msg);
            depth_info_pub.publish(latest_depth_info_msg);
            rgb_info_pub.publish(latest_rgb_info_msg);
        }
    }

private:
    ros::Publisher depth_pub, rgb_pub, depth_info_pub, rgb_info_pub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub, rgb_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub, rgb_info_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    ros::Timer timer;

    sensor_msgs::ImageConstPtr latest_depth_msg;
    sensor_msgs::ImageConstPtr latest_rgb_msg;
    sensor_msgs::CameraInfoConstPtr latest_depth_info_msg;
    sensor_msgs::CameraInfoConstPtr latest_rgb_info_msg;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_sync_publisher");
    ros::NodeHandle nh;
    ImageSyncPublisher isp(nh);
    ros::spin();
    return 0;
}
