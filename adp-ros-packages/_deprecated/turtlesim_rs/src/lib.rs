pub mod turtle;
pub mod turtle_frame;

use rclrs::QoSProfile;

pub fn topic_qos() -> QoSProfile {
    QoSProfile::topics_default().keep_last(7).reliable()
}