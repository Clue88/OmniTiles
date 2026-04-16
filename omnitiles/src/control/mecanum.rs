// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Mecanum wheel inverse kinematics.
//!
//! Wheel layout (top-down view, rollers shown as `/` and `\`):
//!
//! ```text
//!   FL \/  /\ FR        +X (forward)
//!                         ^
//!   RL /\  \/ RR          |  +omega (CCW)
//!                         +---> +Y (left)
//! ```

/// Compute per-wheel speeds from body-frame velocity commands.
///
/// Returns `[front_left, front_right, rear_left, rear_right]`, each in -1.0..1.0.
/// If any raw speed exceeds 1.0, all four are scaled down proportionally to
/// preserve the commanded direction.
pub fn mecanum_ik(vx: f32, vy: f32, omega: f32) -> [f32; 4] {
    // Left wheels are mounted mirrored from the right, so their rollers
    // contribute to vy with the opposite sign.
    let fl = vx - vy - omega;
    let fr = vx - vy + omega;
    let rl = vx + vy - omega;
    let rr = vx + vy + omega;

    let mut speeds = [fl, fr, rl, rr];

    // Proportional desaturation
    let mut max_abs: f32 = 1.0;
    for &s in &speeds {
        let a = if s < 0.0 { -s } else { s };
        if a > max_abs {
            max_abs = a;
        }
    }
    if max_abs > 1.0 {
        for s in &mut speeds {
            *s /= max_abs;
        }
    }

    speeds
}
