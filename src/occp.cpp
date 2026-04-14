void OccupancyGridMap::updatePoseFromTF()
{
    try
    {
        const auto transform = tf_buffer_->lookupTransform(
            "map",
            "base_link",
            tf2::TimePointZero
        );

        const auto &t = transform.transform.translation;
        const auto &q = transform.transform.rotation;

        current_pose[0] = t.x;
        current_pose[1] = t.y;

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        current_pose[2] = yaw;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "TF lookup failed: " << ex.what());
    }
}