use rclrs::{Executor, Node, Publisher};
use rclrs::MandatoryParameter;

use geometry_msgs::msg::Twist;
use turtlesim_msgs::action::RotateAbsolute;

use turtlesim::topic_qos;

use std::io;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

struct TeleopTurtle {
    node: Node,

    twist_pub: Publisher<Twist>,
    rotate_absolute_client: ActionClient<RotateAbsolute>,
    // goal_handle: 
}

impl TeleopTurtle {
    pub fn new(executor: &Executor) -> Self {
        let node = executor.create_node("teleop_turtle").unwrap();
        let scale_angular = node
            .declare_parameter("scale_angular")
            .default(2.0)
            .mandatory()
            .unwrap();
        let scale_linear = node
            .declare_parameter("scale_linear")
            .default(2.0)
            .mandatory()
            .unwrap();
        let twist_pub = node.create_publisher::<Twist>("cmd_vel".qos(topic_qos()));
        let rotate_absolute_client = node.create_action_client("turtle1/rotate_absolute");

        Self {
            node,
            scale_angular,
            scale_linear,
            twist_pub,
            rotate_absolute_client,
        }
    }

    pub fn key_loop(&mut self) {
        println!("Reading from keyboard");
        println!("---------------------------");
        println!("Use arrow keys to move the turtle.");
        println!("Use g|b|v|c|d|e|r|t keys to rotate to absolute orientations. 'f' to cancel a rotation.");
        println!("'q' to quit.");

        let _stdout = io::stdout().into_raw_mode().unwrap();
        let stdin = io::stdin();
        for key in stdin.keys() {
            self.linear = 0.0;
            self.angular = 0.0;
        }
    }
}

impl TeleopTurtle {
    fn spin() {
        
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let mut executor = rclrs::Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("teleop_turtle")?;


}