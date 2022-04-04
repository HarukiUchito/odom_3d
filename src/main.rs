use std::sync::{Arc, Mutex};

use anyhow::Result;
use rclrust::qos::QoSProfile;
use rclrust_msg::geometry_msgs::msg::TwistWithCovarianceStamped;
use rclrust_msg::nav_msgs::msg::Path;
use std::time::Duration;

mod odometry;
use odometry::{EKFBaseTrait, InputVec, Odometry3D};

fn current_time() -> f64 {
    rclrust::Clock::ros().unwrap().now().unwrap().nanosecs as f64 * 1e-9
}

pub fn to_ros_time(time: f64) -> rclrust_msg::builtin_interfaces::msg::Time {
    let sec = f64::floor(time);
    let nsec: u32 = ((time - sec) * 1e9) as u32;
    rclrust_msg::builtin_interfaces::msg::Time {
        sec: sec as i32,
        nanosec: nsec as u32,
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let mut node = ctx.create_node("examples_subscriber")?;

    let odom_arc = Arc::new(Mutex::new(Odometry3D::new()));
    let path_arc = Arc::new(Mutex::new(Path {
        header: rclrust_msg::std_msgs::msg::Header {
            frame_id: "base_link".to_string(),
            stamp: to_ros_time(0.0),
        },
        poses: Vec::new(),
    }));

    let sub_odom = Arc::clone(&odom_arc);
    let _subscription = node.create_subscription(
        "twist",
        move |msg: Arc<TwistWithCovarianceStamped>| {
            let t = current_time();
            let mut odom = sub_odom.lock().unwrap();

            let lin_vel = msg.twist.twist.linear.x;
            let ang_vel = msg.twist.twist.angular.z;
            odom.predict(InputVec::from_vec(vec![t, lin_vel, ang_vel]));
        },
        &QoSProfile::default(),
    )?;

    let odom2 = Arc::clone(&odom_arc);
    let _timer = node.create_wall_timer(Duration::from_millis(100), move || {
        let t = current_time();
        let odom = odom2.lock().unwrap();
        println!("current on t: {}, state: {:?}", t, odom.state());
    });

    node.wait();

    Ok(())
}
