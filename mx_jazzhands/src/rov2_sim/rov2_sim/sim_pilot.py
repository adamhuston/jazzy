"""sim_pilot: a canned velocity source for Milestone 2 sim validation.

Publishes a repeating forward -> arc -> stop pattern of geometry_msgs/Twist on
the framework's command-contract topic (default /rov2_core/cmd_vel). Both peers
subscribe to that topic: the framework routes it to MockActuator (proof of
receipt via /rov2_core/status) and the Isaac Sim rover uses it to move. This
substitutes for a real brain so the command-and-state loop can be exercised.

Sim-only tooling: this package is never included in the hardware build path.
With use_sim_time:=true the publish cadence follows /clock, so it only ticks
once the sim is publishing the clock.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def _control_qos() -> QoSProfile:
    """Mirror rov2_core::qos::control(): reliable, volatile, KeepLast(10)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


class SimPilot(Node):
    def __init__(self):
        super().__init__("sim_pilot")

        self.declare_parameter("cmd_vel_topic", "/rov2_core/cmd_vel")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("forward_sec", 3.0)
        self.declare_parameter("arc_sec", 3.0)
        self.declare_parameter("stop_sec", 2.0)
        self.declare_parameter("loop", True)

        topic = self.get_parameter("cmd_vel_topic").value
        rate = self.get_parameter("publish_rate_hz").value
        lin = self.get_parameter("linear_speed").value
        ang = self.get_parameter("angular_speed").value
        forward_sec = self.get_parameter("forward_sec").value
        arc_sec = self.get_parameter("arc_sec").value
        stop_sec = self.get_parameter("stop_sec").value
        self._loop = self.get_parameter("loop").value

        if rate <= 0.0:
            self.get_logger().warn("publish_rate_hz must be > 0; defaulting to 10 Hz")
            rate = 10.0
        self._dt = 1.0 / rate

        # Phase table: (name, linear.x, angular.z, duration_sec).
        self._phases = [
            ("forward", lin, 0.0, forward_sec),
            ("arc", lin, ang, arc_sec),
            ("stop", 0.0, 0.0, stop_sec),
        ]
        self._phase_idx = 0
        self._elapsed = 0.0
        self._done = False

        self._pub = self.create_publisher(Twist, topic, _control_qos())
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"sim_pilot publishing to {topic} at {rate:.1f} Hz (loop={self._loop})"
        )
        self._log_phase()

    def _log_phase(self):
        name, lin, ang, dur = self._phases[self._phase_idx]
        self.get_logger().info(
            f"phase '{name}': lin.x={lin:.2f} ang.z={ang:.2f} for {dur:.1f}s"
        )

    def _advance(self):
        self._phase_idx += 1
        if self._phase_idx >= len(self._phases):
            if self._loop:
                self._phase_idx = 0
            else:
                self._phase_idx = len(self._phases) - 1
                self._done = True
                self.get_logger().info("pattern complete; holding stop")
                return
        self._log_phase()

    def _tick(self):
        if not self._done:
            self._elapsed += self._dt
            _, _, _, dur = self._phases[self._phase_idx]
            if self._elapsed >= dur:
                self._elapsed = 0.0
                self._advance()

        _, lin, ang, _ = self._phases[self._phase_idx]
        if self._done:
            lin, ang = 0.0, 0.0

        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: leave the rover stopped on exit.
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
