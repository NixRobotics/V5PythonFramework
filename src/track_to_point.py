import math
from collections import deque

def wrap_pi(angle):
    """Wrap angle to [-pi, pi]."""
    a = (angle + math.pi) % (2 * math.pi) - math.pi
    return a

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class GuidanceController:
    def __init__(self,
                 dt=0.02,
                 # lookahead params - L is a scaling factor with distance to target. KL is gain and L_min/Lmax are limits
                 # This is not a PID value, just a lookahead distance calculation
                 k_L=0.6, L_min=0.05, L_max=0.4,

                 # Turn PID: Kp scaled with distance, Kd is constance
                 # distance-weighted angular gain - adjusts Omega (angular velocity) Kp based on distance (closer = smaller)
                 K_far=3.0, K_near=0.6, d0=0.2,
                 # yaw-rate damping - angular velocity implemented as PD controller
                 Kd=0.08,

                 # angular limits
                 omega_abs_limit=3.0,    # rad/s
                 alpha_max=40.0,         # rad/s^2 (angular accel limit)

                 # filtering
                 omega_filter_alpha=0.35,

                 # linear speed scheduling
                 v_max=1.0, # m/s
                 dist_slow_k=0.2, # speed Kp scaling based on distance
                 heading_slow_k=6.0, # speed Kp scaling based on heading
                 v_min_final=0.04, # minimum forward speed when close to target

                 # final pose
                 r_final=0.10, # distance to target
                 K_final=0.25, # final heading Kp for turn cmd
                 omega_final_limit=1.2 # clamp for final angular velocity
                 ):
        self.dt = dt

        # lookahead
        self.k_L = k_L
        self.L_min = L_min
        self.L_max = L_max

        # gains
        self.K_far = K_far
        self.K_near = K_near
        self.d0 = d0
        self.Kd = Kd

        # limits
        self.omega_abs_limit = omega_abs_limit
        self.alpha_max = alpha_max

        # filtering
        self.omega_filter_alpha = omega_filter_alpha
        self.omega_cmd_filtered = 0.0
        self.omega_prev = 0.0

        # speed scheduling
        self.v_max = v_max
        self.dist_slow_k = dist_slow_k
        self.heading_slow_k = heading_slow_k
        self.v_min_final = v_min_final

        # final pose
        self.r_final = r_final
        self.K_final = K_final
        self.omega_final_limit = omega_final_limit

    def _lookahead_point(self, x, y, xg, yg):
        dx = xg - x
        dy = yg - y
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            return xg, yg, 0.0
        L = clamp(self.k_L * dist, self.L_min, self.L_max)
        x_look = x + dx * (L / dist)
        y_look = y + dy * (L / dist)
        return x_look, y_look, dist

    def _distance_weighted_Kp(self, dist):
        return self.K_far * (dist / (dist + self.d0)) + self.K_near

    def _speed_schedule(self, dist, angle_error):
        # distance scaling (smoothly reduces speed near goal)
        S_dist = dist / (dist + self.dist_slow_k)
        # heading error scaling (reduce speed while turning)
        S_heading = 1.0 / (1.0 + self.heading_slow_k * abs(angle_error))
        v = self.v_max * S_dist * S_heading
        # clamp minimum when very close (so final controller can finish)
        if dist < self.r_final:
            v = max(v, self.v_min_final)
        return v

    def step(self, pose, yaw_rate, goal):
        """
        Single control tick.
        pose: (x, y, theta) in meters and radians
        yaw_rate: measured angular rate in rad/s (positive ccw)
        goal: (xg, yg)
        Returns: (v_cmd, omega_cmd, debug_dict) in m/s and rad/s
        """
        x, y, theta = pose
        xg, yg = goal

        # Lookahead and desired heading
        x_look, y_look, dist = self._lookahead_point(x, y, xg, yg)
        target_theta = math.atan2(y_look - y, x_look - x)
        angle_error = wrap_pi(target_theta - theta)

        debug = {
            "dist": dist,
            "target_theta": target_theta,
            "angle_error": angle_error
        }

        # Final-pose mode (gentle final alignment)
        if dist <= self.r_final:
            # Use reduced gain so controller is gentle
            Kp = self.K_final
            raw_omega = Kp * angle_error - self.Kd * yaw_rate
            raw_omega = clamp(raw_omega, -self.omega_final_limit, self.omega_final_limit)
            
            # optionally set v very low or zero; here we keep a small forward speed
            v_cmd = self.v_min_final
        else:
            # Normal guidance
            Kp = self._distance_weighted_Kp(dist)
            raw_omega = Kp * angle_error - self.Kd * yaw_rate
            v_cmd = self._speed_schedule(dist, angle_error)

        debug.update({"Kp": Kp, "raw_omega": raw_omega, "v_cmd_prelimit": v_cmd})

        # Absolute clamp
        omega_cmd = clamp(raw_omega, -self.omega_abs_limit, self.omega_abs_limit)

        # Rate-limit angular acceleration
        max_detla_omega = self.alpha_max * self.dt
        delta_omega = clamp(omega_cmd - self.omega_prev, -max_detla_omega, max_detla_omega)
        omega_cmd = self.omega_prev + delta_omega
        self.omega_prev = omega_cmd

        # Exponential filter on omega_cmd for smoother commands (alpha near 0 more smoothing)
        a = self.omega_filter_alpha
        self.omega_cmd_filtered = a * omega_cmd + (1 - a) * self.omega_cmd_filtered
        omega_cmd_out = self.omega_cmd_filtered

        debug.update({
            "omega_after_clamp": omega_cmd,
            "omega_filtered": omega_cmd_out
        })

        return v_cmd, omega_cmd_out, debug
    
# Differential drive guidance + heading controller (Python)
# Below is a drop-in guidance module you can integrate with your existing PID and tracking classes. It:
# Implements pure-pursuit lookahead with L proportional to distance.
# Produces smooth desired heading ψ_d.
# Uses a distance-weighted proportional law for angular command with yaw-rate damping (uses your yaw-rate sensor).
# Applies absolute ω clamp, angular acceleration rate-limiting, and an exponential filter on ω_cmd.
# Schedules forward speed down with distance and heading error and switches to a final-pose mode close to goal.
# Copy the code and call GuidanceController.step(...) each control tick (dt). Adapt parameter values to tune for your robot.

# How to integrate with your PID and tracking
# If your inner loop uses a PID that commands wheel velocities from v_cmd and omega_cmd, feed v_cmd and omega_cmd as references
# to that PID (or convert to left/right wheel velocities first).
# If you already have a heading PID class, you can bypass raw_omega calculation and instead supply psi_d and let the heading PID
# compute omega_cmd. Still keep rate-limiting and filtering on the PID output to prevent actuator-driven jumps.
# For logging, include these fields each tick: dist, psi_d, angle_error, Kp, raw_omega, omega_after_clamp, omega_filtered, v_cmd.

# dt = 0.02 s
# v_max = 1.0 m/s
# k_L = 0.6, L_min = 0.05 m, L_max = 0.4 m
# K_far = 3.0, K_near = 0.6, d0 = 0.2
# Kd = 0.08
# omega_abs_limit = 3.0 rad/s (≈172°/s)
# alpha_max = 40 rad/s^2
# omega_filter_alpha = 0.35
# r_final = 0.08–0.12 m, K_final = 0.25, omega_final_limit = 1.2 rad/s
