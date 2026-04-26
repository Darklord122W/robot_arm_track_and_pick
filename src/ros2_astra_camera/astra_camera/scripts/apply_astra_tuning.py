#!/usr/bin/env python3
# One-shot tuning node: after the Astra driver is up, locks exposure/gain and
# toggles laser/LDP/fan to settings that reduce depth-image noise on a static
# scene. Exits once all calls complete.
#
# Implementation note: we shell out to `ros2 service call` rather than using
# rclpy clients. The driver runs inside a ComposableNodeContainer, and rclpy
# clients in Humble intermittently report services as available via
# wait_for_service but then hang on call_async. The CLI path matches the
# invocation documented in the driver's README and works reliably.
#
# Order matters: LDP toggle internally restarts streams, so LDP must run
# before exposure/gain — otherwise those settings get wiped.

import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node


# (service_name, type_string, payload_string)
def bool_payload(v):
    return ('std_srvs/srv/SetBool', '{data: ' + ('true' if v else 'false') + '}')


def int_payload(v):
    return ('astra_camera_msgs/srv/SetInt32', '{data: ' + str(int(v)) + '}')


class AstraTuner(Node):
    def __init__(self):
        super().__init__('astra_tuner')
        self.declare_parameter('camera_namespace', 'camera')
        self.declare_parameter('laser_enable', True)
        self.declare_parameter('ldp_enable', False)
        self.declare_parameter('fan_enable', True)
        self.declare_parameter('depth_auto_exposure', False)
        self.declare_parameter('depth_exposure', 2000)
        self.declare_parameter('depth_gain', 200)
        self.declare_parameter('per_call_timeout_sec', 10.0)
        self.declare_parameter('startup_delay_sec', 3.0)

        self.ns = self.get_parameter('camera_namespace').value
        self.per_timeout = float(self.get_parameter('per_call_timeout_sec').value)
        self.startup_delay = float(self.get_parameter('startup_delay_sec').value)

    def _resolve_service(self, name):
        # Try the README path first (/<ns>/<name>). Fall back to the
        # composable-node-name-included path (/<ns>/<name>/<svc>) if the
        # first isn't present.
        candidates = [f'/{self.ns}/{name}', f'/{self.ns}/{self.ns}/{name}']
        try:
            out = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True, text=True, timeout=5.0,
            ).stdout
        except subprocess.TimeoutExpired:
            out = ''
        for c in candidates:
            if c in out.splitlines():
                return c
        # None matched: fall back to the first candidate and let the call
        # surface the error.
        return candidates[0]

    def _call(self, name, type_str, payload):
        full = self._resolve_service(name)
        cmd = ['ros2', 'service', 'call', full, type_str, payload]
        try:
            res = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self.per_timeout,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'{name}: CLI call timed out after {self.per_timeout}s')
            return False
        # `ros2 service call` exits 0 even on service-reported failure; parse
        # the response text for success=True.
        text = (res.stdout or '') + (res.stderr or '')
        ok = 'success=True' in text or 'success=true' in text
        # If the service returns only data (no success field), treat exit 0 as ok.
        if 'success=' not in text and res.returncode == 0 and 'response:' in text:
            ok = True
        status = 'ok' if ok else 'FAIL'
        self.get_logger().info(f'{status} {full} {payload}')
        if not ok:
            self.get_logger().warn(text.strip()[:400])
        return ok

    def _wait_for_service(self, name, total_timeout=30.0):
        full_candidates = [f'/{self.ns}/{name}', f'/{self.ns}/{self.ns}/{name}']
        deadline = time.time() + total_timeout
        while time.time() < deadline:
            try:
                out = subprocess.run(
                    ['ros2', 'service', 'list'],
                    capture_output=True, text=True, timeout=5.0,
                ).stdout.splitlines()
            except subprocess.TimeoutExpired:
                out = []
            for c in full_candidates:
                if c in out:
                    return c
            time.sleep(0.5)
        return None

    def run(self):
        # Give the composable container a moment to register services.
        if self.startup_delay > 0:
            self.get_logger().info(f'Waiting {self.startup_delay:.1f}s for driver startup...')
            time.sleep(self.startup_delay)

        # Sanity check: wait until at least one of our services is visible.
        probe = self._wait_for_service('set_laser_enable', total_timeout=30.0)
        if probe is None:
            self.get_logger().error(
                'No Astra services visible after 30s. Is the driver running? '
                f'Looked for /{self.ns}/set_* and /{self.ns}/{self.ns}/set_*')
            return
        self.get_logger().info(f'Found service (using {probe}); applying tuning...')

        # 1. LDP first — toggling it restarts streams, wiping later settings.
        self._call('set_ldp_enable', *bool_payload(
            self.get_parameter('ldp_enable').value))

        # 2. Laser on — ensure IR projector is active.
        self._call('set_laser_enable', *bool_payload(
            self.get_parameter('laser_enable').value))

        # 3. Fan — reduces thermal drift / wavy bias.
        self._call('set_fan_mode', *bool_payload(
            self.get_parameter('fan_enable').value))

        # 4. Kill auto-exposure before pinning exposure and gain. "depth"
        # services control the IR-sensor device-level property; using "depth"
        # instead of "ir" because enable_ir=false by default, in which case
        # set_ir_* services are never registered.
        self._call('set_depth_auto_exposure', *bool_payload(
            self.get_parameter('depth_auto_exposure').value))
        self._call('set_depth_exposure', *int_payload(
            self.get_parameter('depth_exposure').value))
        self._call('set_depth_gain', *int_payload(
            self.get_parameter('depth_gain').value))

        self.get_logger().info(
            'Astra tuning complete. Sweep depth_exposure (1000-8000) while '
            'viewing /camera/depth/image_raw to find the crispest pattern.')


def main():
    # Make sure ROS envs are inherited by the subprocess calls.
    os.environ.setdefault('RCUTILS_COLORIZED_OUTPUT', '1')
    rclpy.init()
    node = AstraTuner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
