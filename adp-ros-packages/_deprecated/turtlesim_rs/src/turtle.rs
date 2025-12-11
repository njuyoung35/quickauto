use std::time;
use std::sync::{Arc, Mutex};
use std::sync::mpsc::{channel, Receiver};

use rclrs::{Node, Subscription, Publisher, Service, ActionServer, ActionGoalReceiver, Time, IntoPrimitiveOptions};

use geometry_msgs::msg::Twist;
use turtlesim_rs_msgs::msg::{Pose, Color};
use turtlesim_rs_msgs::srv::{
    SetPen, SetPen_Request, SetPen_Response,
    TeleportAbsolute, TeleportAbsolute_Request, TeleportAbsolute_Response,
    TeleportRelative, TeleportRelative_Request, TeleportRelative_Response,
};
// use turtlesim_rs_msgs::action::RotateAbsolute;

// use std::os::raw::{c_int, c_double};
// use qt_core::QPointF;
// use qt_gui::{QImage, QPainter, QPen, QColor, QRgba64};
// use cpp_core::CastInto;
use std::f32::consts::{FRAC_PI_2, PI};

use eframe::egui::{Image, Pos2, Rect, Ui, Vec2};
use tiny_skia::{LineCap, Paint, PathBuilder, Pixmap, Stroke, Transform};

use crate::turtle_frame::{TURTLE_IMG_HEIGHT, TURTLE_IMG_WIDTH};

const DEFAULT_PEN_R: u8 = 0xb3;
const DEFAULT_PEN_G: u8 = 0xb8;
const DEFAULT_PEN_B: u8 = 0xff;
const DEFAULT_PEN_ALPHA: u8 = 255;
const DEFAULT_STROKE_WIDTH: f32 = 3.0;

enum TurtleSrvs {
    SetPen(u8, u8, u8, u8, u8),
    TeleportAbsolute(f32, f32, f32),
    TeleportRelative(f32, f32),
}

struct TurtleVel {
    lin_vel: f64,
    ang_vel: f64,
    last_command_time: Time,
}

pub struct Pen<'a> {
    paint: Paint<'a>,
    stroke: Stroke,
}

pub struct Turtle<'a> {
    node: Node,
    image: Image<'a>,
    pos: Pos2,
    orient: f32,
    meter: f32,
    turtle_vel: Arc<Mutex<TurtleVel>>,
    pen_on: bool,
    pen: Pen<'a>,

    srv_rx: Receiver<TurtleSrvs>,

    velocity_sub: Subscription<Twist>,
    pose_pub: Publisher<Pose>,
    color_pub: Publisher<Color>,
    set_pen_srv: Service<SetPen>,
    teleport_absolute_srv: Service<TeleportAbsolute>,
    teleport_relative_srv: Service<TeleportRelative>,
    // rotate_absolute_action_server: ActionServer<RotateAbsolute>,

    // rotate_absolute_goal_handle: ActionGoalReceiver<RotateAbsolute>,
    // rotate_absolute_feedback: Arc<RotateAbsolute::Feedback>,
    // rotate_absolute_result: Arc<RotateAbsolute::Result>,
    // rotate_absolute_start_orient: f64,
}

impl<'a> Turtle<'a> {
    pub fn new(node: Node, real_name: &str, image: Image<'a>, pos: Pos2, orient: f32) -> Self {
        let qos = crate::topic_qos();
        let meter = TURTLE_IMG_HEIGHT;
        let turtle_vel = Arc::new(Mutex::new(TurtleVel {
            lin_vel: 0.0,
            ang_vel: 0.0,
            last_command_time: node.get_clock().now(),
        }));
        let pen_on = true;
        let stroke = Stroke {
            width: DEFAULT_STROKE_WIDTH,
            line_cap: LineCap::Round,
            ..Default::default()
        };
        let mut paint = Paint::default();
        paint.set_color_rgba8(
            DEFAULT_PEN_R,
            DEFAULT_PEN_G,
            DEFAULT_PEN_B,
            DEFAULT_PEN_ALPHA,
        );
        paint.anti_alias = true;
        let pen = Pen { paint, stroke };

        let turtle_vel_clone = turtle_vel.clone();
        let node_clone = node.clone();
        let velocity_sub = node.create_subscription(
            format!("{real_name}/cmd_vel").qos(qos),
            move |msg: Twist| {
                let mut vel = turtle_vel_clone.lock().unwrap();
                vel.lin_vel = msg.linear.x;
                vel.ang_vel = msg.angular.z;
                vel.last_command_time = node_clone.get_clock().now();
            },
        ).unwrap();
        let pose_pub = node.create_publisher::<Pose>(format!("{real_name}/pose").qos(qos)).unwrap();
        let color_pub = node.create_publisher::<Color>(format!("{real_name}/color_sensor").qos(qos)).unwrap();

        let (srv_tx, srv_rx) = channel();

        let set_pen_srv_tx = srv_tx.clone();
        let set_pen_srv = node.create_service::<SetPen, _>(
            format!("{real_name}/set_pen").qos(qos),
            move |req: SetPen_Request| {
                let (r, g, b, width, off) = (req.r, req.g, req.b, req.width, req.off);
                set_pen_srv_tx
                    .send(TurtleSrvs::SetPen(r, g, b, width, off))
                    .unwrap();
                SetPen_Response::default()
            },
        ).unwrap();

        let teleport_absolute_srv_tx = srv_tx.clone();
        let teleport_absolute_srv = node.create_service::<TeleportAbsolute, _>(
            format!("{real_name}/teleport_absolute").qos(qos),
            move |req: TeleportAbsolute_Request| {
                let (x, y, theta) = (req.x, req.y, req.theta);
                teleport_absolute_srv_tx
                    .send(TurtleSrvs::TeleportAbsolute(x, y, theta))
                    .unwrap();
                TeleportAbsolute_Response::default()
            },
        ).unwrap();

        let teleport_relative_srv_tx = srv_tx.clone();
        let teleport_relative_srv = node.create_service::<TeleportRelative, _>(
            format!("{real_name}/teleport_relative").qos(qos),
            move |req: TeleportRelative_Request| {
                let (linear, angular) = (req.linear, req.angular);
                teleport_relative_srv_tx
                    .send(TurtleSrvs::TeleportRelative(linear, angular))
                    .unwrap();
                TeleportRelative_Response::default()
            },
        ).unwrap();

        // let rotate_absolute_action_server = node.create_action_server::<RotateAbsolute>(
        //     format!("{real_name}/rotate_absolute").qos(qos),
        //     move |req: RequestedGoal<RotateAbsolute>| {
        //         let mut goal_handle = match req.try_accept() {
        //             Ok(goal_handle) => goal_handle,
        //             Err(err) => return req.terminate(),
        //         };


        //     },
        // ).unwrap();
        Self {
            node,
            image,
            pos,
            orient,
            meter,
            turtle_vel,
            pen_on,
            pen,

            srv_rx,

            velocity_sub,
            pose_pub,
            color_pub,
            set_pen_srv,
            teleport_absolute_srv,
            teleport_relative_srv,
            // rotate_absolute_action_server,
        }
    }

    pub fn rotate_image(&mut self) {
        let image = self.image.clone();
        self.image = image.rotate(-self.orient + FRAC_PI_2, Vec2::splat(0.5));
    }

    pub fn update(
        &mut self,
        dt: f64,
        path_image: &mut Pixmap,
        canvas_width: f32,
        canvas_height: f32,
    ) -> bool {
        let mut modified = false;

        let old_orient = self.orient;
        let old_pos = self.pos;

        unsafe {
            modified |= self.handle_service_requests(path_image, old_pos, canvas_height);
        }

        let mut turtle_vel = self.turtle_vel.lock().unwrap();
        let is_old_command = self
            .node
            .get_clock()
            .now()
            .compare_with(&turtle_vel.last_command_time, |now_ns, command_ns| {
                let diff_ns = (now_ns - command_ns) as u64;
                time::Duration::from_nanos(diff_ns) > time::Duration::from_secs(1)
            })
            .unwrap();

        if is_old_command {
            turtle_vel.lin_vel = 0.0;
            turtle_vel.ang_vel = 0.0;
        }

        let lin_vel = turtle_vel.lin_vel;
        let ang_vel = turtle_vel.ang_vel;

        drop(turtle_vel);

        self.orient += (ang_vel * dt) as f32;
        // Keep orient between -pi and +pi
        self.orient -= 2.0 * PI * ((self.orient + PI) / (2.0 * PI)).floor();

        self.pos.x += self.orient.cos() * (lin_vel * dt) as f32;
        self.pos.y += -self.orient.sin() * (lin_vel * dt) as f32;

        // Clamp to screen size
        if self.pos.x < 0.0
            || self.pos.x > canvas_width
            || self.pos.y < 0.0
            || self.pos.y > canvas_height
        {
            println!(
                "Oh no! I hit the wall! (Clamping from [x={}, y={}])",
                self.pos.x, self.pos.y
            );
        }

        self.pos.x = f32::min(f32::max(self.pos.x, 0.0), canvas_width);
        self.pos.y = f32::min(f32::max(self.pos.y, 0.0), canvas_height);

        let pose_msg = Pose {
            x: self.pos.x,
            y: canvas_height - self.pos.y,
            theta: self.orient,
            linear_velocity: lin_vel as f32,
            angular_velocity: ang_vel as f32,
        };

        self.pose_pub.publish(pose_msg).unwrap();

        let pixel_color = path_image.pixel(
            (self.pos.x * self.meter) as u32,
            (self.pos.y * self.meter) as u32,
        );

        if let Some(color) = pixel_color {
            let color_msg = turtlesim_rs_msgs::msg::Color {
                r: color.red(),
                g: color.green(),
                b: color.blue(),
            };
            self.color_pub.publish(color_msg).unwrap();
        }

        if self.orient != old_orient {
            modified = true;
            self.rotate_image();
        }

        if self.pos != old_pos {
            modified = true;

            if self.pen_on {
                self.draw_line_on_path_image(path_image, old_pos, self.pos);
            }
        }

        modified
    }

    pub unsafe fn handle_service_requests(&mut self, path_image: &mut Pixmap, old_pos: Pos2, canvas_height: f32) -> bool {
        let mut modified = false;

        for srvs in self.srv_rx.try_iter() {
            match srvs {
TurtleSrvs::SetPen(r, g, b, width, off) => {
                    self.pen.paint.set_color_rgba8(r, g, b, 255);
                    self.pen.stroke.width = width as f32;
                    self.pen_on = off == 0;
                },
                TurtleSrvs::TeleportAbsolute(x, y, theta) => {
                    self.pos.x = x;
                    self.pos.y = canvas_height - y;
                    self.orient = theta;
                    self.draw_line_on_path_image(path_image, old_pos, self.pos);
                    modified = true;
                },
                TurtleSrvs::TeleportRelative(linear, angular) => {
                    self.orient += angular;
                    self.pos.x += self.orient.cos() * linear;
                    self.pos.y += -self.orient.sin() * linear;
                    self.draw_line_on_path_image(path_image, old_pos, self.pos);
                    modified = true;
                },
            }
        }

        modified
    }

    pub fn paint(&self, ui: &mut Ui) {
        let top_left_pos = Pos2 {
            x: self.pos.x * self.meter - TURTLE_IMG_WIDTH / 2.0,
            y: self.pos.y * self.meter - TURTLE_IMG_HEIGHT / 2.0,
        };
        let image_rect =
            Rect::from_min_size(top_left_pos, Vec2::new(TURTLE_IMG_WIDTH, TURTLE_IMG_HEIGHT));
        self.image.paint_at(ui, image_rect);
    }

    fn draw_line_on_path_image(&self, path_image: &mut Pixmap, pos1: Pos2, pos2: Pos2) {
        let mut path_builder = PathBuilder::new();
        path_builder.move_to(pos1.x * self.meter, pos1.y * self.meter);
        path_builder.line_to(pos2.x * self.meter, pos2.y * self.meter);

        if let Some(path) = path_builder.finish() {
            path_image.stroke_path(
                &path,
                &self.pen.paint,
                &self.pen.stroke,
                Transform::identity(),
                None,
            );
        }
    }
}

// pub fn rotate_absolute_accept_callback(handle: ActionGoalReceiver<RotateAbsolute>) {

// }