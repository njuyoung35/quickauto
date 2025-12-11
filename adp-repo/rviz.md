## ROS 2 Humble documentation

### tf2

### URDF

### RViz user guide

- rosindex
  - rviz2
  - rviz\_assimp\_vendor
  - rviz\_common
  - rviz\_default\_plugins
  - rviz\_ogre\_vendor
  - rviz\_rendering
  - rviz\_rendering\_tests
  - rviz\_visual\_testing\_framework

rviz2 : 단순 시각화 위주 담당 (ogre + opengl) gazebo : 실제 물리 시뮬레이션 연산까지 담당 (물리엔진 ODE, bullet, simbody, DART, etc)

#### Displays

- display
- display properties : 말그대로 객체 속성 같은거
- display status : 통신 상 상태.OK, Warning, Error, Disabled

##### Built-in display types

- Axes : 참조 frame 기준으로 축 보여줌.
  - \[description\] Displays a set of Axes
- Effort
  - \[description\] Shows the effort being put into each revolute joint of a robot
    - \[messages used\] sensor\_msgs/msg/JointStates
- Camera
  - \[description\] Creates a new rendering window from the perspective of a camera, and overlays the image on top of it.
  - \[messages used\] sensor\_msgs/msg/Image, sensor\_msgs/msg/CameraInfo
- Grid
  - \[description\] Displays a 2D or 3D grid along a plane
- Grid Cells
  - \[description\] Draws cells from a grid, usually obstacles from a costmap from the navigation stack.
  - \[messages used\] nav\_msgs/msg/GridCells
- Image
  - \[description\] Creates a new rendering window with an Image. Unlike the Camera display, this display does not use a CameraInfo
  - \[messages used\] sensor\_msgs/msg/Image
- InteractiveMarker
  - \[description\] Displays 3D objects from one or multiple Interactive Marker servers and allows mouse interaction with them
  - \[messages used\] visualization\_msgs/msg/InteractiveMarker
- Laser Scan
  - \[description\] Shows data from a laser scan, with different options for rendering modes, accumulation, etc.
  - \[messages used\] sensor\_msgs/msg/LaserScan
- Map
  - \[description\] Displays a map on the ground plane.
  - \[messages used\] nav\_msgs/msg/OccupancyGrid
- Markers
  - \[description\] Allows programmers to display arbitrary primitive shapes through a topic
    - \[messages used\] visualization\_msgs/msg/Marker, visualization\_msgs/msg/MarkerArray
- Path
  - \[description\] Shows a path from the navigation stack.
  - \[messages used\] nav\_msgs/msg/Path
- Point
  - \[description\] Draws a point as a small sphere.
  - \[messages used\] geometry\_msgs/msg/PointStamped
- Pose
  - \[description\] Draws a pose as either an arrow or axes.
  - \[messages used\] geometry\_msgs/msg/PoseStamped
- Pose Array
  - \[description\] Draws a “cloud” of arrows, one for each pose in a pose array
  - \[messages used\] geometry\_msgs/msg/PoseArray
- Point Cloud(2)
  - \[description\] Shows data from a point cloud, with different options for rendering modes, accumulation, etc.
  - \[messages used\] sensor\_msgs/msg/PointCloud, sensor\_msgs/msg/PointCloud2
- Polygon
  - \[description\] Draws the outline of a polygon as lines.
  - \[messages used\] geometry\_msgs/msg/Polygon
- Odometry
  - \[description\] Accumulates odometry poses from over time.
  - \[messages used\] nav\_msgs/msg/Odometry
- Range
  - \[description\] Displays cones representing range measurements from sonar or IR range sensors. Version: Electric+
  - \[messages used\] sensor\_msgs/msg/Range
- RobotModel
  - \[description\] Shows a visual representation of a robot in the correct pose (as defined by the current TF transforms).
- TF
  - \[description\] Displays the tf2 transform hierarchy.
- Wrench
  - \[description\] Draws a wrench as arrow (force) and arrow + circle (torque)
  - \[messages used\] geometry\_msgs/msg/WrenchStamped
- Twist
  - \[description\] Draws a twist as arrow (linear) and arrow + circle (angular)
  - \[messages used\] geometry\_msgs/msg/TwistStamped

##### 추가된거

- AccelStamped
- DepthCloud
- FluidPressure
- Illuminance
- PointCloud
- PoseWithCovariance
- RelativeHumidity
- Temperature

##### 상세 설명

- Laser Scan : 라이다 스캔 데이터 /scan 표시해줌.
- Camera : 실제 차량에 부착되는 카메라가 보내주는 데이터를 세계 또는 판넬, 공간?에 띄워줌
- TF : 로봇을 구성하는 링크들을 시각화. 프레임과 프레임이 연결되고 어떤 관계를 맺는지, 축은 어떻게 되는지 보여줄 것.. 속성 Frames 아래에 프레임들이 나열되어 있음.

#### Configurations

- Displays + their properties
- Tool properties
- The viewpoint and settings for the 3D visualization

#### Views Panel

- orbital camera (default)
- FPS camera
- top-down orghographic
- XY orbit
- third person follower
- custom views

#### Coordinate Frames

- fixed frame (world, map 등)
- target frame

#### Tools

- interact
- move camera
- select
- focus camera
- measure
- 2d pose esimate : ROS topic으로 `initialpose` 보내서 약속된 초기화를 시킨다.
  - ros2 navigation2 스택과 잘 들러붙는다.
- 2d nav goal : 클릭한 좌표를 `goal_pose` ROS topic으로 설정해 통신한다. 드래그해서 방향도 정해주기
- publish point : 클릭한 객체의 프레임 좌표를 `clicked_point`에 담아 publish한다.

#### Time
