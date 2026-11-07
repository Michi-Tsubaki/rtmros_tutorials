"""Validate live sensor streams and capture initial state for XML launches."""
import math
import time


ARMS = {'larm': 'left', 'rarm': 'right'}


def detect_arms(timeout=5.0, skip_compensated=False, with_initial_state=False):
    """Observe actual fresh raw samples, rather than advertised topic names."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import WrenchStamped
    from sensor_msgs.msg import JointState

    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('admittance_sensor_detection', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    seen = {}
    joint_sample = [None]
    compensated = {}

    def receive(arm, msg):
        age = (node.get_clock().now().nanoseconds -
               (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        w = msg.wrench
        values = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)
        if (-.1 <= age < .25 and msg.header.frame_id == arm.upper()+'_FTSENSOR_SENSOR_Link'
                and all(math.isfinite(v) for v in values)):
            seen[arm] = time.monotonic()

    def joint_receive(msg):
        if len(msg.name) == len(msg.position) and all(math.isfinite(v) for v in msg.position):
            joint_sample[0] = msg

    try:
        if with_initial_state:
            node.create_subscription(JointState, '/joint_states', joint_receive, qos_profile_sensor_data)
        for arm, side in ARMS.items():
            if with_initial_state:
                node.create_subscription(WrenchStamped, f'/{side}_ft/force_torque_compensated',
                                         lambda msg, arm=arm: compensated.update({arm: msg}), qos_profile_sensor_data)
            node.create_subscription(WrenchStamped, f'/{side}_ft/force_torque',
                                     lambda msg, arm=arm: receive(arm, msg),
                                     qos_profile_sensor_data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        arms = [arm for arm in ARMS if time.monotonic()-seen.get(arm, 0.) < .25]
        if not arms:
            raise RuntimeError('No live force sensors: start the FT drivers/bridge before this launch')
        node.get_logger().info('Detected force sensors: ' + ', '.join(arms))
        if skip_compensated:
            arms = [arm for arm in arms if node.count_publishers(
                f'/{ARMS[arm]}_ft/force_torque_compensated') == 0]
        if with_initial_state:
            sample = joint_sample[0]
            def fresh(msg):
                stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                return -.1 <= (node.get_clock().now().nanoseconds-stamp)/1e9 < .25
            if sample is None or not fresh(sample):
                raise RuntimeError('Need fresh fullbody joint feedback before bringup')
            state = dict(zip(sample.name, sample.position))
            required = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1'] + [
                f'{arm}_JOINT{i}' for arm in ('LARM', 'RARM') for i in range(6)]
            if not all(name in state for name in required):
                raise RuntimeError('Incomplete fullbody joint feedback')
            for arm in arms:
                msg = compensated.get(arm)
                if msg is None or not fresh(msg) or msg.header.frame_id != arm.upper()+'_FTSENSOR_SENSOR_Link':
                    raise RuntimeError('Need fresh compensated FT for '+arm+'; start ft_compensation first')
                w = msg.wrench
                values = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Non-finite compensated FT for '+arm)
                for axis, v in zip(('force.x','force.y','force.z','torque.x','torque.y','torque.z'), values):
                    state[ARMS[arm]+'_ft_sensor/'+axis] = v
            return arms, state
        return arms
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()



def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['admittance', 'admittance_bringup', 'ft_compensation'])
    parser.add_argument('--left-calibration-file')
    parser.add_argument('--right-calibration-file')
    args = parser.parse_args()
    detected = detect_arms(skip_compensated=args.kind == 'ft_compensation',
                           with_initial_state=args.kind == 'admittance_bringup')
    arms, initial_state = detected if args.kind == 'admittance_bringup' else (detected, {})
    if args.kind == 'ft_compensation':
        from nextage_tutorials.ft_payload import load_calibration
        for arm in arms:
            path = getattr(args, ARMS[arm] + '_calibration_file')
            load_calibration(path, arm.upper() + '_FTSENSOR_SENSOR_Link', 'WAIST')
    # stdout is consumed once by the XML launch's command substitution.
    print(repr({'arms': arms, 'initial_state': initial_state}))
