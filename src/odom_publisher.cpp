#include "motor_broker.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void MotorBroker::OdometryTFPublisher()
{
    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_msg = nav_msgs::msg::Odometry();

    m_init_pose_pub
      = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 5);
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

    this->declare_parameter("/init_pose/x", 0.f);
    this->declare_parameter("/init_pose/y", 0.f);
    this->declare_parameter("/init_pose/theta", 0.f);
    this->declare_parameter("/publish_tf_odom", true);

    publishInitialPose();

    auto base_link_id = "base_link"; // tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = "odom";           // tf::resolve(ros::this_node::getNamespace(), "odom");
    odom_trans.header.frame_id = odom_id;
    odom_trans.child_frame_id = base_link_id;
    odom_msg.pose.pose.position.z = 0;

    this->get_parameter("/publish_tf_odom", m_publish_tf_odom);
}

void MotorBroker::publishInitialPose()
{
    float init_x, init_y, init_theta;
    this->get_parameter("/init_pose/x", init_x);
    this->get_parameter("/init_pose/y", init_y);
    this->get_parameter("/init_pose/theta", init_theta);

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg
      = geometry_msgs::msg::PoseWithCovarianceStamped();
    init_pose_msg.pose.pose.position.x = init_x;
    init_pose_msg.pose.pose.position.y = init_y;
    init_pose_msg.pose.pose.position.z = 0;
    init_pose_msg.pose.covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    init_pose_msg.pose.covariance[0] = 0.1f; // X covariance
    init_pose_msg.pose.covariance[7] = 0.1f; // Y covariance
    init_pose_msg.pose.covariance[35] = 0.2; // Rz covariance
    tf2::Quaternion quat_tf_orientation;
    quat_tf_orientation.setRPY(0, 0, init_theta);
    quat_tf_orientation.normalize();
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf_orientation);
    init_pose_msg.pose.pose.orientation = quat_msg;

    init_pose_msg.header.frame_id = "map";
    init_pose_msg.header.stamp = this->now();

    m_init_pose_pub->publish(init_pose_msg);
}

/**
 * @brief OdometryTFPublisher::publishOdom publish a full-fledged odom message. It takes too long
 * for rosserial to publish it directly
 * @param odom_lighter_msg the partial message published by rosserial (even lighter than odom_light)
 */
void MotorBroker::publishOdom(krabi_msgs::msg::OdomLighter odom_lighter_msg)
{

    odom_msg.pose.pose.position.x = odom_lighter_msg.pose_x;
    odom_msg.pose.pose.position.y = odom_lighter_msg.pose_y;
    tf2::Quaternion odom_orientation_quat;
    odom_orientation_quat.setRPY(0, 0, odom_lighter_msg.angle_rz);
    odom_msg.pose.pose.orientation = tf2::toMsg(odom_orientation_quat);
    publishTf(odom_msg.pose.pose);

    odom_msg.header.stamp = odom_lighter_msg.header.stamp;
    odom_msg.header.stamp = this->now();

    odom_msg.twist.twist.linear.x = odom_lighter_msg.speed_vx;
    odom_msg.twist.twist.angular.z = odom_lighter_msg.speed_wz;

    m_odom_pub->publish(odom_msg);
}

void MotorBroker::publishTf(const geometry_msgs::msg::Pose& pose)
{
    if (!m_publish_tf_odom)
    {
        return;
    }

    odom_trans.header.stamp = this->now();

    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation = pose.orientation;

    // send the transform
    m_tf_broadcaster->sendTransform(odom_trans);
}
