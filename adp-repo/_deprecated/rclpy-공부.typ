== Clock

내용 없음

== Initialization, Shutdown, and Spinning

일반적인 ROS 프로그램이 따르는 연산 절차:

+ 초기화
  - ```python rclpy.init(args, context, domain_id, ..)```
  - ```rust Context::new(args, options: InitOptions)```
    - `InitOptions`는 domain_id와 관련된 것.
    - `Context`는 DDS의 추상화라고 보면 될 듯.
  - ```rust Context::default_from_env()```
+ 하나 이상의 ROS node 만들기
  - 파이썬의 경우 `rclpy.create_node()`로 맥락에서 바로 노드를 만들 수도 있고, `rclpy.get_global_executor()`를 통해 얻은 `Executor` 객체를 통해 `add_node()`로 추가할 수 있다.
  - ```rust ctx.create_executor(runtime: impl ExecutorRuntime)```
    - `ExecutorRuntime`에는 `channel`, `spin`, `spin_async`의 메소드가 포함되어 있다.
  - ```rust ctx.create_basic_executor()```
+ node 콜백 통신하기
  - 노드를 만들고 나서는, 파이썬 기준, `spin(node, executor)`, `spin_once(node, executor, timeout_sec)`, `spin_until_future_complete(node, future, executor, timeout_sec)`를 통해 콜백을 통신할 수 있다.
    - 우리가 개념으로 배운 토픽, 서비스, 액션은 api 호출상 직접적으로 드러나지는 않는 것으로 보인다. 이것은 단순히 통신 패턴에 불과한 것으로 보임.
    - api 상에서는 `Executor.spin`의 변종을 통해 3가지 경우 말고, 더 디테일한 통신이 가능한 것으로 보인다.
+ shutdown
  - ```python rclpy.shutdown()```

```
[실행 방법]          [아키텍처]
ros2 run single     [Process] → [Executor] → [Single Node]
                   
ros2 run composite  [Process] → [Executor] → [Node1, Node2, Node3]
                   
ros2 launch system  [Process A] → [Executor] → [Nav Nodes]
                    [Process B] → [Executor] → [Perception Nodes]  
                    [Process C] → [Executor] → [Control Nodes]
```

== Node

내용 없음

하술할 토픽, 서비스, 액션에 대해 그 인터페이스, 메시지 형식에 대한 내용은 api 상에 없고, 그걸 주고 받는 두 주체에 대한 설명만 나온다.

각각에 대하여 Publisher - Subscriber, Server - Client, Action Server - Action Client의 관계로 묘사된다.

파이썬에서는 클래스로 구현되어 있고, 러스트에서는 type alias로 별칭된 `Arc<T>`로 보인다.

== Topics

=== Publisher

- `publisher_impl`
- `msg_type`
- `topic`
- `qos_profile`

=== Subscriber

- `subscriber_impl`
- `msg_type`
- `topic`
- `callback`
- `callback_group`
- `qos_profile`
- `raw`

== Services

=== Client

- `context`
- `client_impl`
- `srv_type`
- `srv_name`
- `qos_profile`
- `callback_group`

=== Server

- `server_impl`
- `srv_type`
- `srv_name`
- `callback`
- `callback_group`
- `qos_profile`

== Actions

=== Action Client

- `node`
- `action_type`
- `action_name`
- `callback_group`
- `goal_service_qos_profile`
- `result_service_qos_profile`
- `cancel_service_qos_profile`
- `feedback_sub_qos_profile`
- `status_sub_qos_profile`

=== Action Server

내용 없음

== Time

== Timer

== Parameter

== Logging

== Context

== Execution and Callbacks

콜백 실행을 컨트롤하는 두 컴포넌트가 있다: executors와 callback groups

- executor는 콜백에 대한 실질적 실행을 책임진다.
- callback group은 콜백들에 대한 동기성 규칙을 강제한다.

callback group은 메소드들에 entity라는 것을 인자로 받는데 이것은 subscription, timer, client, service 같은 waitable instance를 말한다.

== Utilities

== QoS

= rclrs 0.6.0

== Context

기본적인 rclrs의 사용법은 다음과 같다:

```rust
use rclrs::*;

let context = Context::default_from_env()?;
let mut executor = context.create_basic_executor();
let node = executor.create_node("example_node")?;

let subscription = node.create_subscription(
    "topic_name",
    |msg: example_interfaces::msg::String| {
        println!("Received message: {}", msg.data);
    }
)?;

executor.spin(SpinOptions::default()).first_error()?;
```

만약 특정 상태 데이터와 상호작용하는 콜백이 필요하다면 `Worker`를 사용해봐라. 특히 그 상태데이터가 다른 콜백과 공유되어야 할 때:

```rust
// This worker will manage the data for us.
// The worker's data is called its payload.
let worker = node.create_worker::<Option<String>>(None);

// We use the worker to create a subscription.
// This subscription's callback can borrow the worker's
// payload with its first function argument.
let subscription = worker.create_subscription(
    "topic_name",
    |data: &mut Option<String>, msg: example_interfaces::msg::String| {
        // Print out the previous message, if one exists.
        if let Some(previous) = data {
            println!("Previous message: {}", *previous)
        }

        // Save the latest message, to be printed out the
        // next time this callback is triggered.
        *data = Some(msg.data);
    }
)?;
```

== Executor

== Node

rclrs의 `Node`는 type alias로, `Arc<NodeState>`이다. 그러므로 구조체 `NodeState`의 api를 호출할 수 있는 것이다.

이외에도 `Publisher`, `Subscription`, `Service`, `Client`, `ActionServer`, `ActionClient`, `Timer`, `Worker` 등도 전부 type alias이며, 관련된 `~State` 구조체에 api가 달려 있다.

== Publisher

```rust node.create_publisher(options: impl Into<PublisherOptions>)```

== Subscription

== Service

== Client

== Worker (for async)

== ExecutorCommands (for async)

== rust의 타입 시스템에 대하여..

```rust
use example_interfaces::msg::String;
use std_msgs::msg::String as StringMsg;
```

러스트는 패키지, 노드, 심지어 workspace?까지도 넘나들며 메시지들의 타입을 정적으로 검사하는 것으로 보인다. with rosidl_generator_rs + rosidl_runtime_rs

지금 ros::example_interfaces에 문제가 있는 것으로 보이는데 이름에 'example'이 들어가지만 중요한 것이라고 하는 것으로 보임