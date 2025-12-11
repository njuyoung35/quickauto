// https://github.com/autowarefoundation/autoware_universe/blob/main/control/autoware_trajectory_follower_base/include/autoware/trajectory_follower_base/longitudinal_controller_base.hpp

use ssu2_control_msgs::msg::Longitudinal;
use crate::input_data::InputData;
use crate::sync_data::LongitudinalSyncData;
use crate::control_horizon::LongitudinalHorizon;

struct LongitudinalOutput {
    control_cmd: Longitudinal,
    control_cmd_horizon: LongitudinalHorizon,
    sync_data: LongitudinalSyncData,
}

pub struct LongitudinalControllerBase {
}