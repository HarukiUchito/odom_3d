extern crate nalgebra as na;
use na::{SMatrix, SVector};

pub trait EKFBaseTrait<const INPUT_NUM: usize> {
    fn predict(&mut self, input: SVector<f64, INPUT_NUM>);
    fn update(&mut self, input: SVector<f64, INPUT_NUM>);
}

pub struct EKF<const STATE_NUM: usize> {
    x: SVector<f64, STATE_NUM>,
    p: SMatrix<f64, STATE_NUM, STATE_NUM>,
    f: SMatrix<f64, STATE_NUM, STATE_NUM>,
}

impl<const STATE_NUM: usize, const INPUT_NUM: usize> EKFBaseTrait<INPUT_NUM> for EKF<STATE_NUM> {
    fn predict(&mut self, _input: SVector<f64, INPUT_NUM>) {
        let mut p = &self.p;
        let f = &self.f;
        p = &(f * p * f.transpose());
    }
    fn update(&mut self, _input: SVector<f64, INPUT_NUM>) {
        println!("base update");
    }
}

pub trait EKFTrait<const INPUT_NUM: usize>: EKFBaseTrait<INPUT_NUM> {
    fn set_f(&mut self);
    fn state_update(&mut self, input: SVector<f64, INPUT_NUM>);
}

// odometry 3D implementation

const STATE_NUM: usize = 6;
const INPUT_NUM: usize = 3;

pub type InputVec = SVector<f64, INPUT_NUM>;
type Vec6 = SVector<f64, STATE_NUM>;
type Mat6 = SMatrix<f64, STATE_NUM, STATE_NUM>;
pub struct Odometry3D {
    last_time: f64,
    pub ekf: EKF<STATE_NUM>,
}

impl EKFBaseTrait<INPUT_NUM> for Odometry3D {
    fn predict(&mut self, input: InputVec) {
        self.ekf.predict(input);
        self.state_update(input);
    }
    fn update(&mut self, input: InputVec) {
        self.ekf.update(input);
    }
}

impl EKFTrait<INPUT_NUM> for Odometry3D {
    fn set_f(&mut self) {
        todo!();
    }

    /// ekf state update by motion model
    /// * `input` - input vector contains f64 values
    ///             [current_t lin_vel ang_vel]
    fn state_update(&mut self, input: InputVec) {
        let t = input[0];
        if self.last_time < 0.0 {
            // first input
            self.last_time = t;
            return;
        }

        let dt = t - self.last_time;
        let lin_vel = input[1];
        let ang_vel = input[2];
        let theta = self.ekf.x[5];
        self.ekf.x += Vec6::from_vec(vec![
            lin_vel * theta.cos(),
            lin_vel * theta.sin(),
            0.0,
            0.0,
            0.0,
            ang_vel * dt,
        ]);
    }
}

impl Odometry3D {
    pub fn new() -> Odometry3D {
        Odometry3D {
            last_time: -1.0,
            ekf: EKF {
                x: Vec6::zeros(),
                p: Mat6::zeros(),
                f: Mat6::zeros(),
            },
        }
    }

    pub fn state(&self) -> Vec6 {
        self.ekf.x
    }

    pub fn covariance(&self) -> Mat6 {
        self.ekf.p
    }
}
