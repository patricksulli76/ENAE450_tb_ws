#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node        import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan

class GapFollower(Node):
    """
    Reactive maze‑solver.

    1.  Median‑filters the scan (robust to speckles)
    2.  Builds a binary “free / occupied” array at WARNING_DIST
    3.  Picks the *widest* free corridor → gap centre gives desired heading
    4.  Angular speed ∝ heading error, linear speed ∝ free space in front
    """

    # ── parameters you will actually tweak ──────────────────────────
    MAX_LIN  = 0.1         # [m/s]
    MAX_ANG  = 0.2         # [rad/s]
    WARNING  = 0.45          # [m] start steering round walls
    STOP     = 0.25          # [m] emergency stop
    MIN_GAP  = 20            # [deg] smallest corridor worth taking
    MED_WIN  = 3             # rolling‑median window (odd integer)
    # ────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('gap_follower')
        self.pub  = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub  = self.create_subscription(LaserScan, 'scan', self.cb, 10)

    # ---------- helper utilities ------------------------------------
    @staticmethod
    def median(ranges, k):
        a = np.array(ranges, dtype=np.float32)
        a[np.isinf(a) | np.isnan(a)] = 10.0          # treat bad reads as “far”
        if k < 2: return a
        pad = np.pad(a, (k//2, k//2), 'edge')
        return np.array([np.median(pad[i:i+k]) for i in range(len(a))])

    @staticmethod
    def i2deg(i, n):                       # index → degrees (front = 0 °)
        return (i * 360.0) / n

    # ---------- core algorithm --------------------------------------
    def widest_gap(self, r):
        free = r > self.WARNING
        gaps, start = [], None
        for i, ok in enumerate(free):
            if ok and start is None:
                start = i
            elif not ok and start is not None:
                gaps.append((start, i-1)); start = None
        if start is not None:                        # wrap‑around gap
            gaps.append((start, len(r)-1))
        if not gaps: return None
        s, e = max(gaps, key=lambda g: g[1]-g[0])
        width_deg = self.i2deg(e - s + 1, len(r))
        if width_deg < self.MIN_GAP: return None
        return (s + e)//2, width_deg

    def cb(self, scan: LaserScan):
        r = self.median(scan.ranges, self.MED_WIN)

        # ── emergency stop if something < STOP directly ahead ──
        if np.min(r[:5]+r[-5:]) < self.STOP:
            self.publish(0.0, 0.8*self.MAX_ANG)
            return

        gap = self.widest_gap(r)
        if gap is None:                               # nothing safe → spin search
            self.publish(0.0, 0.5*self.MAX_ANG)
            return

        centre_i, _ = gap
        err_deg = self.i2deg(centre_i, len(r))
        if err_deg > 180: err_deg -= 360              # shortest‑turn convention
        err_rad = np.deg2rad(err_deg)

        ang = np.clip(2.0*err_rad, -self.MAX_ANG, self.MAX_ANG)
        lin = np.clip(0.6*r[0], 0.0, self.MAX_LIN)    # faster if corridor is deep
        self.publish(lin, ang)

    def publish(self, lin, ang):
        twist = Twist();  twist.linear.x = lin;  twist.angular.z = ang
        self.pub.publish(twist)

# ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GapFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
