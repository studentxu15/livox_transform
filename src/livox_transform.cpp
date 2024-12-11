#include "header.h"
#include <livox_transform/TestConfig.h>

class LivoxTransform : public ParamServer
{
public:
    ros::Subscriber sub_livox_raw;
    ros::Publisher pub_livox_points;

    boost::recursive_mutex reconfigure_mutex;

    dynamic_reconfigure::Server<livox_transform::TestConfig> server;
    dynamic_reconfigure::Server<livox_transform::TestConfig>::CallbackType f_c;

    double tran_x;
    double tran_y;
    double tran_z;
    double theta_x;
    double theta_y;
    double theta_z;

    Eigen::Matrix4f transform_mat;

    pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud_;

    LivoxTransform()
        : server(reconfigure_mutex)
    {
        sub_livox_raw = nh.subscribe(livox_raw_message_name_, 10, &LivoxTransform::livox_raw_callback, this);

        pub_livox_points = nh.advertise<sensor_msgs::PointCloud2>(livox_points_message_name_, 10);

        f_c = boost::bind(&LivoxTransform::dynamicReconfigureCallback, this, _1, _2);
        server.setCallback(f_c);

        allocateMemory();
    }

    void allocateMemory()
    {
        raw_cloud_ = boost::make_shared<pcl::PointCloud<PointXYZIRT>>();
        // 初始化动态参数
        livox_transform::TestConfig config;
        config.tran_x = livox_TR_[0];
        config.tran_y = livox_TR_[1];
        config.tran_z = livox_TR_[2];
        config.theta_x = livox_TR_[3];
        config.theta_y = livox_TR_[4];
        config.theta_z = livox_TR_[5];
        {
            boost::recursive_mutex::scoped_lock lock(reconfigure_mutex);
            server.updateConfig(config);
        }

        tran_x = livox_TR_[0];
        tran_y = livox_TR_[1];
        tran_z = livox_TR_[2];
        theta_x = livox_TR_[3];
        theta_y = livox_TR_[4];
        theta_z = livox_TR_[5];
        transform_mat = Eigen::Matrix4f::Identity();
        Computer_trans_mat();
    }

    void Computer_trans_mat()
    {
        Eigen::Matrix4f rotation_x;
        rotation_x << 1, 0, 0, 0,
            0, std::cos(theta_x), -std::sin(theta_x), 0,
            0, std::sin(theta_x), std::cos(theta_x), 0,
            0, 0, 0, 1;

        Eigen::Matrix4f rotation_y;
        rotation_y << std::cos(theta_y), 0, std::sin(theta_y), 0,
            0, 1, 0, 0,
            -std::sin(theta_y), 0, std::cos(theta_y), 0,
            0, 0, 0, 1;

        Eigen::Matrix4f rotation_z;
        rotation_z << std::cos(theta_z), -std::sin(theta_z), 0, 0,
            std::sin(theta_z), std::cos(theta_z), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        transform_mat = rotation_z * rotation_y * rotation_x;

        transform_mat(0, 3) = tran_x;
        transform_mat(1, 3) = tran_y;
        transform_mat(2, 3) = tran_z;
    }

    void livox_raw_callback(const livox_ros_driver::CustomMsg::ConstPtr &lidar_msg)
    {
        raw_cloud_->clear();
        for (const auto &p : lidar_msg->points)
        {
            RTPoint point;
            int line_num = (int)p.line;

            Eigen::Vector4f point_vector(p.x, p.y, p.z, 1.0);
            // 应用变换
            Eigen::Vector4f transformed_vector = transform_mat * point_vector;

            point.x = transformed_vector(0);
            point.y = transformed_vector(1);
            point.z = transformed_vector(2);
            point.intensity = p.reflectivity;
            point.time = p.offset_time / 1e9;
            point.ring = (line_num);

            raw_cloud_->push_back(point);
        }

        sensor_msgs::PointCloud2 pcd_msg;
        pcl::toROSMsg(*raw_cloud_, pcd_msg);
        pcd_msg.header.stamp = lidar_msg->header.stamp;
        pcd_msg.header.frame_id = livox_frame_;
        pub_livox_points.publish(pcd_msg);
    }

    void dynamicReconfigureCallback(livox_transform::TestConfig &config, uint32_t level)
    {
        tran_x = config.tran_x;
        tran_y = config.tran_y;
        tran_z = config.tran_z;
        theta_x = config.theta_x;
        theta_y = config.theta_y;
        theta_z = config.theta_z;

        ROS_INFO("Reconfigure Request: \ntran_x: %f, \ntran_y: %f, \ntran_z: %f, \ntheta_x: %f, \ntheta_y: %f, \ntheta_z: %f",
                 tran_x, tran_y, tran_z, theta_x, theta_y, theta_z);
        Computer_trans_mat();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_transform");
    LivoxTransform LT;

    ROS_INFO("\033[1;32m----> livox transform Started.\033[0m");

    ros::spin();

    return 0;
}