"""Unit tests for the PARALLEL (oblique) mapping of the sim messenger.

The mapping is the real driver's (CommandFromTwist in
ranger_base/src/motion_logic.cpp): stateless, and the motion it
commands has the signs of the twist's x and y in every case.
"""

import math

from ranger_mini_v3_sim_messenger.sim_messenger import (
    compute_wheel_commands_parallel,
    MAX_STEER_PARALLEL,
    WHEEL_RADIUS,
)


def _approx(a, b, tol=1e-6):
    return abs(a - b) < tol


def _sign(v):
    return (v > 0) - (v < 0)


def _motion(angle, speed):
    return speed * math.cos(angle), speed * math.sin(angle)


# ---------- One row per sign case (same table as the driver) ----------
def test_sign_case_table():
    """Angle and speed per sign case; motion has the twist's signs."""
    rows = [
        # (x, y, angle, speed)
        (0.05, 0.05, math.atan(1.0), math.hypot(0.05, 0.05)),
        (-0.05, -0.05, math.atan(1.0), -math.hypot(0.05, 0.05)),
        (-0.05, 0.05, -math.atan(1.0), -math.hypot(0.05, 0.05)),
        (0.05, -0.05, -math.atan(1.0), math.hypot(0.05, 0.05)),
        (0.0, 0.05, MAX_STEER_PARALLEL, 0.05),
        (0.0, -0.05, MAX_STEER_PARALLEL, -0.05),
    ]
    for x, y, angle, speed in rows:
        wc, used_angle, used_speed = compute_wheel_commands_parallel(x, y)
        assert _approx(used_angle, angle), (x, y)
        assert _approx(used_speed, speed), (x, y)
        mx, my = _motion(used_angle, used_speed)
        assert _sign(my) == _sign(y), (x, y)
        if x != 0.0:
            assert _sign(mx) == _sign(x), (x, y)
        else:
            # 1.570 rad is 0.8 mrad short of 90 degrees: tiny forward residue
            assert abs(mx) <= abs(used_speed) * math.cos(MAX_STEER_PARALLEL) + 1e-12


def test_backward_diagonal_goes_to_the_commanded_side():
    """x<0, y>0 is backward-left: negative angle, negative speed."""
    _, angle, speed = compute_wheel_commands_parallel(-0.3, 0.3)
    mx, my = _motion(angle, speed)
    assert mx < 0 and my > 0


def test_all_wheels_identical():
    wc, angle, speed = compute_wheel_commands_parallel(0.3, 0.3)
    for steer in (wc.steer_fl, wc.steer_fr, wc.steer_rl, wc.steer_rr):
        assert _approx(steer, math.pi / 4)
    for vel in (wc.vel_fl, wc.vel_fr, wc.vel_rl, wc.vel_rr):
        assert _approx(vel, math.hypot(0.3, 0.3) / WHEEL_RADIUS)


def test_zero_of_either_sign_gives_the_same_result():
    """-0.0 and +0.0 for x give the same pure-sideways command."""
    a = compute_wheel_commands_parallel(0.0, 0.3)
    b = compute_wheel_commands_parallel(-0.0, 0.3)
    assert a[1] == b[1] and a[2] == b[2]


def test_steering_wraps_at_pure_sideways():
    """A tiny x of the other sign asks for -90 instead of +90 degrees."""
    _, plus, _ = compute_wheel_commands_parallel(1e-6, 0.05)
    _, minus, _ = compute_wheel_commands_parallel(-1e-6, 0.05)
    assert _approx(plus, MAX_STEER_PARALLEL)
    assert _approx(minus, -MAX_STEER_PARALLEL)


def test_no_result_depends_on_earlier_commands():
    """Stateless: the same twist gives the same command in any order."""
    first = compute_wheel_commands_parallel(0.0, 0.3)
    compute_wheel_commands_parallel(-0.3, 0.3)
    compute_wheel_commands_parallel(-0.3, -0.3)
    again = compute_wheel_commands_parallel(0.0, 0.3)
    assert first[1:] == again[1:]


# ---------- Stationary (everything zero) ----------
def test_zero_command_yields_zero_wheels():
    wc, angle, speed = compute_wheel_commands_parallel(0.0, 0.0)
    assert speed == 0.0
    assert wc.vel_fl == 0.0
    assert wc.steer_fl == 0.0  # default WheelCommands has zeros
