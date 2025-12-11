#[derive(Default, Clone, Copy)]
struct PidParams {
    kp: f64,
    ki: f64,
    kd: f64,
    max_ret_p: f64,
    min_ret_p: f64,
    max_ret_i: f64,
    min_ret_i: f64,
    max_error_integral: f64,
    min_error_integral: f64,
    max_ret_d: f64,
    min_ret_d: f64,
    max_ret: f64,
    min_ret: f64,
}

pub struct PidController {
    params: PidParams,
    error_integral: f64,
    prev_error: f64,
    is_first_time: bool,
    is_gains_set: bool,
    is_limits_set: bool,
}

impl PidController {
    pub fn new() -> Self {
        Self {
            params: PidParams::default(),
            error_integral: 0.0,
            prev_error: 0.0,
            is_first_time: true,
            is_gains_set: false,
            is_limits_set: false,
        }
    }

    pub fn calculate(&mut self, error: f64, dt: f64, enable_integration: bool, pid_contributions: &mut [f64, f64, f64]) {
        if !self.is_gains_set || !self.is_limits_set {
            panic!("Trying to calculate uninitialized PID");
        }
        let p = &self.params;

        let mut ret_p = p.kp * error;
        ret_p = ret_p.clamp(p.min_ret_p, p.max_ret_p);

        if enable_integration {
            self.error_integral += error * dt;
            self.error_integral = self.error_integral.clamp(p.min_error_integral, p.max_error_integral);
        }
        let ret_i = p.ki * self.error_integral;

        let mut error_differential;
        if self.is_first_time {
            error_differential = 0.0;
            self.is_first_time = false;
        } else {
            error_differential = (error - self.prev_error) / dt;
        }
        let mut ret_d = p.kd * error_differential;
        ret_d = ret_d.clamp(p.min_ret_d, p.max_ret_d);

        self.prev_error = error;

        pid_contributions[0] = ret_p;
        pid_contributions[1] = ret_i;
        pid_contributions[2] = ret_d;

        ret = (ret_p + ret_i + ret_d).clamp(p.min_ret, p.max_ret);
        ret
    }

    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.params.kp = kp;
        self.params.ki = ki;
        self.params.kd = kd;
        self.max_error_integral = self.params.max_ret_i / self.params.ki;
        self.min_error_integral = self.params.min_ret_i / self.params.ki;
        self.is_gains_set = true;
    }

    pub fn set_limits(&mut self,
        max_ret_p: f64, min_ret_p: f64, max_ret_i: f64, min_ret_i: f64,
        max_ret_d: f64, min_ret_d: f64, max_ret: f64, min_ret: f64) {
        self.params.max_ret_p = max_ret_p;
        self.params.min_ret_p = min_ret_p;
        self.params.max_ret_i = max_ret_i;
        self.params.min_ret_i = min_ret_i;
        self.params.max_ret_d = max_ret_d;
        self.params.min_ret_d = min_ret_d;
        self.params.max_ret = max_ret;
        self.params.min_ret = min_ret;
        if self.is_gains_set {
            self.max_error_integral = self.params.max_ret_i / self.params.ki;
            self.min_error_integral = self.params.min_ret_i / self.params.ki;
        }
        self.is_limits_set = true;
    }

    pub fn reset(&mut self) {
        self.error_integral = 0.0;
        self.prev_error = 0.0;
        self.is_first_time = true;
    }
}