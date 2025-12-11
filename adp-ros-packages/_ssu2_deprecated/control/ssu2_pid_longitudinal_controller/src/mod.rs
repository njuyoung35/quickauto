pub mod control_horizon {
    use ssu2_control_msgs::msg::Lateral;
    use ssu2_control_msgs::msg::Longitudinal;

    pub struct LateralHorizon {
        time_step_ms: f64,
        controls: Vec<Lateral>,
    }

    pub struct LongitudinalHorizon {
        time_step_ms: f64,
        controls: Vec<Longitudinal>,
    }
}

pub mod input_data {
    use ssu2_planning_msgs::msg::Trajectory;
    use nav_msgs::msg::Odometry;
    use ssu2_vehicle_msgs::msg::SteeringReport;
    use geometry_msgs::msg::AccelWithCovarianceStamped;
    // use ssu2_api_msgs::msg::OperationModeState;

    pub struct InputData {
        current_trajectory: Trajectory,
        current_odometry: Odometry,
        current_steering: SteeringReport,
        current_accel: AccelWithCovarianceStamped,
        // current_operation_mode: OperationModeState,
    }
}

pub mod sync_data {
    #[derive(Default, Clone, Copy)]
    pub struct LateralSyncData {
        is_steer_converged: bool,
    }

    #[derive(Default, Clone, Copy)]
    pub struct LongitudinalSyncData {

    }   
}