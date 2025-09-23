import threading
import time
from collections import deque
import matplotlib
matplotlib.use('TkAgg')  # Add this BEFORE importing pyplot
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy 
import rclpy
from rclpy.node import Node
from ati_sensor_interfaces.msg import ForceTorque
from ati_sensor_interfaces.srv import GetForceTorque


WINDOW_S = 20.0
PLOT_HZ = 60

class AtiService(Node):

    def __init__(self):
        super().__init__('ati_service')
        self.service_ = self.create_service(GetForceTorque, 'get_force_torque', self.get_force_torque_callback)
        self.latest_message = None
        self.subscription = self.create_subscription(ForceTorque, 'force_torque', self.listener_callback, 10)
        self.buf = {
            't': deque(),
            'fx': deque(), 'fy': deque(), 'fz': deque(),
            'tx': deque(), 'ty': deque(), 'tz': deque()
        }

    def get_force_torque_callback(self, request, response):
        if self.latest_message is not None:
            response.msg = self.latest_message
        return response

    def listener_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.latest_message = msg

        self.buf['t'].append(now)
        self.buf['fx'].append(msg.fx); self.buf['fy'].append(msg.fy); self.buf['fz'].append(msg.fz)
        self.buf['tx'].append(msg.tx); self.buf['ty'].append(msg.ty); self.buf['tz'].append(msg.tz)

        while self.buf['t'] and now - self.buf['t'][0] > WINDOW_S:
            for q in self.buf.values():
                q.popleft()

    def destroy_node(self):
        self.subscription.destroy()
        super().destroy_node()

def run_plot(node: AtiService):
    fig, (ax_f, ax_t) = plt.subplots(2, 1, figsize=(20, 10), sharex=True)
    ax_f.set_title('Forces [N]')
    ax_t.set_title('Torques [N·m]')
    for ax in (ax_f, ax_t):
        ax.set_xlim(-WINDOW_S, 0)
        ax.grid(True)

    (l_fx,) = ax_f.plot([], [], label='Fx'); (l_fy,) = ax_f.plot([], [], label='Fy'); (l_fz,) = ax_f.plot([], [], label='Fz')
    (l_tx,) = ax_t.plot([], [], label='Tx'); (l_ty,) = ax_t.plot([], [], label='Ty'); (l_tz,) = ax_t.plot([], [], label='Tz')
    ax_f.legend(); ax_t.legend()

    def update(_):
        if not node.buf['t']:
            return l_fx, l_fy, l_fz, l_tx, l_ty, l_tz
        buf_copy = {k: list(v) for k, v in node.buf.items()}

        t0 = buf_copy['t'][-1]
        relt = [ts - t0 for ts in buf_copy['t']]

        # NEW: Calculate indices of visible points
        visible_mask = [(-WINDOW_S <= x <= 0) for x in relt]

        # Update plots with filtered data
        l_fx.set_data(relt, buf_copy['fx'])
        l_fy.set_data(relt, buf_copy['fy'])
        l_fz.set_data(relt, buf_copy['fz'])
        l_tx.set_data(relt, buf_copy['tx'])
        l_ty.set_data(relt, buf_copy['ty'])
        l_tz.set_data(relt, buf_copy['tz'])

        # MODIFIED: Calculate limits using only visible data
        for ax, fields in ((ax_f, ('fx', 'fy', 'fz')), (ax_t, ('tx', 'ty', 'tz'))):
            visible_vals = [v for f in fields for i, v in enumerate(buf_copy[f]) if visible_mask[i]]
            if visible_vals:
                lo, hi = min(visible_vals), max(visible_vals)
                pad = 0.1 * max(1e-3, hi - lo)
                ax.set_ylim(lo - pad, hi + pad)

        return l_fx, l_fy, l_fz, l_tx, l_ty, l_tz

    ani = animation.FuncAnimation(fig, update, interval=1000 / PLOT_HZ, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    ati_service = AtiService()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ati_service,), daemon=True)
    ros_thread.start()

    try:
        run_plot(ati_service)
    finally:
        if rclpy.ok():
            ati_service.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()