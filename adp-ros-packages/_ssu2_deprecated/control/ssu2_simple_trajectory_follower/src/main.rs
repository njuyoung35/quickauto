use rclrs::*;

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("controller")?;

    let pub_ = node.create_publisher::<nav_msgs::msg::Odometry>(
        "odom".qos(QOS_PROFILE_DEFAULT),
    );
    executor.spin(SpinOptions::default()).first_error()
}