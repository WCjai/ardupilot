#include "AP_EmergencyDescent.h"

#if AP_EMERGENCYDESCENT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// Parameter defaults are the values measured and validated in the
// emergency-desent.py prototype. See EMG_DESCENT_NOTES.md section 0.7 for the
// origin of each. Do not "tidy" these without re-validating in SITL.
const AP_Param::GroupInfo AP_EmergencyDescent::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Emergency descent enable
    // @Description: Enable the native emergency-descent mission command pair. When disabled the ENTRY/TARGET mission commands are rejected.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_EmergencyDescent, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: AIRSPEED
    // @DisplayName: Descent airspeed
    // @Description: Target airspeed held through the descent. Also used as the speed floor for the turn-rate to bank conversion.
    // @Units: m/s
    // @Range: 5 60
    // @User: Standard
    AP_GROUPINFO("AIRSPEED", 1, AP_EmergencyDescent, _airspeed, 22.0f),

    // @Param: NAV_GAIN
    // @DisplayName: Proportional navigation gain
    // @Description: Proportional-navigation constant N applied to the line-of-sight rate in the lateral guidance law.
    // @Range: 1 6
    // @User: Advanced
    AP_GROUPINFO("NAV_GAIN", 2, AP_EmergencyDescent, _nav_gain, 3.0f),

    // @Param: PURSUIT_GN
    // @DisplayName: Pursuit gain
    // @Description: Pursuit term applied to bearing error in the lateral guidance law. Raising this too far causes roll oscillation (PIO).
    // @Units: 1/s
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("PURSUIT_GN", 3, AP_EmergencyDescent, _pursuit_gain, 0.8f),

    // @Param: TURN_RATE
    // @DisplayName: Maximum turn rate
    // @Description: Commanded turn rate is clamped to this before conversion to bank angle.
    // @Units: deg/s
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("TURN_RATE", 4, AP_EmergencyDescent, _turn_rate, 25.0f),

    // @Param: ROLL_LIM
    // @DisplayName: Descent roll limit
    // @Description: Maximum bank angle commanded during the descent. Also clamped by the airframe roll limit.
    // @Units: deg
    // @Range: 10 60
    // @User: Advanced
    AP_GROUPINFO("ROLL_LIM", 5, AP_EmergencyDescent, _roll_lim, 45.0f),

    // @Param: PITCH_MIN
    // @DisplayName: Descent minimum pitch
    // @Description: Most nose-down pitch commanded during the descent.
    // @Units: deg
    // @Range: -80 0
    // @User: Advanced
    AP_GROUPINFO("PITCH_MIN", 6, AP_EmergencyDescent, _pitch_min, -60.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Descent maximum pitch
    // @Description: Most nose-up pitch commanded during the descent.
    // @Units: deg
    // @Range: 0 30
    // @User: Advanced
    AP_GROUPINFO("PITCH_MAX", 7, AP_EmergencyDescent, _pitch_max, 15.0f),

    // @Param: GAMMA_P
    // @DisplayName: Flight-path angle gain
    // @Description: Gain driving the flight-path angle onto the line-of-sight elevation in the vertical guidance law.
    // @Range: 0.5 4
    // @User: Advanced
    AP_GROUPINFO("GAMMA_P", 8, AP_EmergencyDescent, _gamma_p, 1.6f),

    // @Param: LOCK_DIST
    // @DisplayName: Terminal lock distance
    // @Description: Slant range at which lateral steering authority is bled off, because the line-of-sight rate goes singular at the target.
    // @Units: m
    // @Range: 5 50
    // @User: Advanced
    AP_GROUPINFO("LOCK_DIST", 9, AP_EmergencyDescent, _lock_dist, 20.0f),

    // Indices 10-12 previously held ENTRY_RAD/ALIGN_TOL/MIN_RNG_R for the
    // entry-gate/alignment phases. Removed when the descent was changed to
    // dive directly from the current position onto a single target -- see
    // AP_EmergencyDescent.h. Left unused rather than reassigned.

    // @Param: DIVE_THR
    // @DisplayName: Descent throttle
    // @Description: Throttle held through the descent, 0 to 1. The prototype descended at idle.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DIVE_THR", 13, AP_EmergencyDescent, _dive_thr, 0.0f),

    // @Param: RATE_HZ
    // @DisplayName: Guidance rate
    // @Description: The guidance law runs at this fixed sub-rate rather than the full loop rate, so line-of-sight-rate differentiation is not amplified by loop-rate noise. All gains were tuned at 10 Hz.
    // @Units: Hz
    // @Range: 5 50
    // @User: Advanced
    AP_GROUPINFO("RATE_HZ", 14, AP_EmergencyDescent, _rate_hz, 10),

    // @Param: MIN_ALT
    // @DisplayName: Minimum start altitude
    // @Description: A descent commanded below this height above home is rejected. Below this there is no room to fly the profile at all.
    // @Units: m
    // @Range: 5 200
    // @User: Standard
    AP_GROUPINFO("MIN_ALT", 15, AP_EmergencyDescent, _min_alt, AP_EMG_MIN_START_ALT_M),

    // @Param: DIVE_PITCH
    // @DisplayName: Aggressive dive-in pitch
    // @Description: Forced nose-down pitch target for the first EMG_DIVE_TIME seconds of the descent, overriding the normal line-of-sight vertical law so the aircraft pitches down immediately instead of gradually steepening as range closes. Only takes effect if EMG_DIVE_TIME is non-zero. Still bounded by EMG_PITCH_MIN.
    // @Units: deg
    // @Range: -80 0
    // @User: Advanced
    AP_GROUPINFO("DIVE_PITCH", 16, AP_EmergencyDescent, _dive_pitch, -70.0f),

    // @Param: DIVE_TIME
    // @DisplayName: Aggressive dive-in duration
    // @Description: How long, from the start of the descent, to force EMG_DIVE_PITCH before handing off to the ordinary line-of-sight vertical law. 0 disables the forced dive-in entirely (line-of-sight law runs from the start, as in the original validated behaviour).
    // @Units: s
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("DIVE_TIME", 17, AP_EmergencyDescent, _dive_time, 0.0f),

    // @Param: VDIVE_EN
    // @DisplayName: Variable-angle body-rate dive enable
    // @Description: Command pitch as a body ROTATION RATE toward the line-of-sight-derived target flight-path angle (the same target the ordinary vertical law tracks -- shallow far out, steepening as range closes) instead of converging on it as an angle target -- the same rate-control mechanism ACRO mode uses. Angle-based attitude control (the ordinary vertical law, and EMG_DIVE_PITCH above) converges through the normal fixed-wing attitude controller, which was found empirically to have an unexplained achieved-pitch ceiling well short of its own commanded target, and which in any case becomes unreliable approaching +-90 deg (Euler angles are singular there). Body-rate control has neither problem. The target angle is capped at EMG_VDIVE_PITCH as the steepest allowed -- it does not dive to a fixed angle regardless of geometry, which would miss the target entirely (no altitude/time budget left for lateral correction once diving near-vertically). Self-regulating -- no separate duration parameter, it simply commands less rate as the target angle is approached.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VDIVE_EN", 18, AP_EmergencyDescent, _vdive_enable, 0),

    // @Param: VDIVE_PITCH
    // @DisplayName: Variable-angle dive steepest pitch
    // @Description: The steepest flight-path angle the body-rate dive is allowed to command, e.g. -85 permits nearly vertical when the geometry calls for it. The actual commanded target is always the line-of-sight-derived angle (shallow far out, steepening as range closes), clamped to this -- not a fixed dive angle. Only takes effect if EMG_VDIVE_EN is enabled.
    // @Units: deg
    // @Range: -90 -45
    // @User: Advanced
    AP_GROUPINFO("VDIVE_PITCH", 19, AP_EmergencyDescent, _vdive_pitch, -85.0f),

    // @Param: VDIVE_RATE_P
    // @DisplayName: Variable-angle dive rate gain
    // @Description: Commanded pitch rate is this gain times the remaining flight-path-angle error to the current target (deg/s per deg), bounded by EMG_VDIVE_RMAX. Higher converges faster but with less margin before the rate bound saturates.
    // @Range: 0.5 6
    // @User: Advanced
    AP_GROUPINFO("VDIVE_RATE_P", 20, AP_EmergencyDescent, _vdive_rate_p, 3.0f),

    // @Param: VDIVE_RMAX
    // @DisplayName: Variable-angle dive max rate
    // @Description: Commanded pitch rate is bounded to this, regardless of how far off the current target angle the aircraft is.
    // @Units: deg/s
    // @Range: 10 90
    // @User: Advanced
    AP_GROUPINFO("VDIVE_RMAX", 21, AP_EmergencyDescent, _vdive_max_rate, 45.0f),

    // @Param: SPIRAL_EN
    // @DisplayName: Positioning orbit enable
    // @Description: Fly to the target and orbit it while descending before releasing into the dive, instead of diving from wherever the aircraft happens to be. A steep dive only reaches the target if the aircraft is nearly above it, and from an arbitrary trigger point it cannot turn onto a steep path fast enough -- it overflies. Orbiting first puts the aircraft within one radius of the target while still high, so the dive that follows is steep by geometry rather than by force. Costs the time of the transit and orbit before the descent begins.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPIRAL_EN", 22, AP_EmergencyDescent, _spiral_enable, 0),

    // @Param: SPIRAL_RAD
    // @DisplayName: Positioning orbit radius
    // @Description: Radius of the positioning orbit. Also sets how steep the following dive can be: released from this radius at height h, the line-of-sight elevation is atan(h/radius), so a smaller radius permits a steeper dive but demands more bank to fly.
    // @Units: m
    // @Range: 30 300
    // @User: Advanced
    AP_GROUPINFO("SPIRAL_RAD", 23, AP_EmergencyDescent, _spiral_radius, 60.0f),

    // @Param: SPIRAL_PTCH
    // @DisplayName: Positioning orbit pitch
    // @Description: Pitch held during the descending spiral. Keep this shallow. Turning needs the lift vector to have a horizontal component, and past roughly 45 deg nose-down the aircraft is near enough vertical that bank rotates it about an axis aimed at the ground rather than curving its path -- measured, at -55 deg the commanded orbit radius had no effect on the trajectory whatsoever. A steep value therefore does not give a fast spiral, it gives a plain steep dive that wanders off the target.
    // @Units: deg
    // @Range: -35 0
    // @User: Advanced
    AP_GROUPINFO("SPIRAL_PTCH", 24, AP_EmergencyDescent, _spiral_pitch, -15.0f),

    // @Param: SPIRAL_CONV
    // @DisplayName: Spiral convergence
    // @Description: Orbit radius per metre of remaining height, so the helix tightens onto the target as it descends instead of holding a fixed circle. A fixed radius leaves a horizontal offset equal to that radius still to be flown off at the bottom, and there is not enough height left to convert it -- measured as a 49 m miss from a 40 m fixed orbit. Radius is clamped to EMG_SPIRAL_RAD at the top and to a few metres at the bottom, so the spiral ends on the target.
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("SPIRAL_CONV", 25, AP_EmergencyDescent, _spiral_conv, 0.30f),

    AP_GROUPEND
};

AP_EmergencyDescent *AP_EmergencyDescent::_singleton;

AP_EmergencyDescent::AP_EmergencyDescent()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_EmergencyDescent must be singleton");
    }
    _singleton = this;
    _phase = Phase::INACTIVE;
}

void AP_EmergencyDescent::init()
{
    _phase = Phase::INACTIVE;
    _last_output = Output{};
}

const char *AP_EmergencyDescent::phase_name() const
{
    switch (_phase) {
    case Phase::INACTIVE: return "INACTIVE";
    case Phase::DESCEND:  return "DESCEND";
    case Phase::IMPACT:   return "IMPACT";
    case Phase::COMPLETE: return "COMPLETE";
    case Phase::ABORTED:  return "ABORTED";
    case Phase::SPIRAL:   return "SPIRAL";
    }
    return "?";
}

bool AP_EmergencyDescent::start(const Location &target, const Location &current,
                                char *reason, uint8_t reason_len)
{
    if (reason != nullptr && reason_len > 0) {
        reason[0] = '\0';
    }

    if (!_enable) {
        if (reason) {
            hal.util->snprintf(reason, reason_len, "EMG disabled");
        }
        return false;
    }

    _target = target;
    _start_range_m = current.get_distance(target);

    if (_start_range_m < 1.0f) {
        if (reason) {
            hal.util->snprintf(reason, reason_len, "already at target");
        }
        return false;
    }

    // Capture the altitude from where the aircraft actually is. An emergency
    // descent starts from whatever height you happen to have.
    int32_t alt_cm = 0;
    if (!current.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_cm)) {
        if (reason) {
            hal.util->snprintf(reason, reason_len, "no altitude");
        }
        return false;
    }
    _start_alt_agl_m = alt_cm * 0.01f;

    if (_start_alt_agl_m < _min_alt) {
        if (reason) {
            hal.util->snprintf(reason, reason_len, "too low (%.0fm)", (double)_start_alt_agl_m);
        }
        return false;
    }

    _phase = _spiral_enable ? Phase::SPIRAL : Phase::DESCEND;
    _phase_start_ms = AP_HAL::millis();
    _spiral_reached_ms = 0;
    _last_guidance_ms = 0;
    _have_prev_los = false;
    _closest_slant_m = FLT_MAX;
    _have_locked_roll = false;
    _last_output = Output{};

    if (_phase == Phase::SPIRAL) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "EMG descent: orbit then dive, %.0fm out, from %.0fm",
                      (double)_start_range_m, (double)_start_alt_agl_m);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "EMG descent: diving onto target, %.0fm out, from %.0fm",
                      (double)_start_range_m, (double)_start_alt_agl_m);
    }
    return true;
}

AP_EmergencyDescent::Output AP_EmergencyDescent::update(const State &state)
{
    if (!is_active()) {
        return Output{};
    }

    const uint32_t now_ms = AP_HAL::millis();

    // Rate-limit the guidance law. mode->update() runs at full loop rate, but
    // every gain here was tuned at 10 Hz and differentiating the line-of-sight
    // angle at 400 Hz amplifies noise ~40x for no benefit. Between ticks we
    // simply re-issue the previous output.
    const float rate_hz = MAX(_rate_hz.get(), 1);
    const uint32_t interval_ms = (uint32_t)(1000.0f / rate_hz);
    if (_last_guidance_ms != 0 && (now_ms - _last_guidance_ms) < interval_ms) {
        return _last_output;
    }
    const float dt = (_last_guidance_ms == 0) ? (1.0f / rate_hz)
                                              : (now_ms - _last_guidance_ms) * 0.001f;
    _last_guidance_ms = now_ms;

    Output out;
    const float range_to_target = state.current.get_distance(_target);
    const float los_deg = wrap_360(degrees(state.current.get_bearing(_target)));
    const float brg_err = wrap_180(los_deg - state.heading_deg);

    switch (_phase) {

    case Phase::SPIRAL: {
        // Position first, dive second. Orbit the target while descending until
        // established overhead, so that the dive which follows is released from
        // within one orbit radius of the target -- at which point the
        // line-of-sight elevation is steep on its own and the ordinary dive law
        // commands a near-vertical attitude that actually reaches the target.
        out.action = Action::ORBIT;
        out.nav_target = _target;
        out.throttle = constrain_float(_dive_thr, 0.0f, 1.0f);

        // Pitch as a RATE, not an angle. Driven as an angle target the steep
        // spiral pitch is capped around -20 deg by the vehicle's angle
        // controller, which leaves the helix descending at only ~8 m/s -- the
        // spiral then works but is far too slow. Rate control reaches the
        // commanded attitude, and the descent rate with it.
        out.pitch_rate_dps = constrain_float(_vdive_rate_p * (_spiral_pitch - (state.pitch_cd * 0.01f)),
                                             -_vdive_max_rate, _vdive_max_rate);
        out.pitch_cd = _spiral_pitch * 100.0f;   // telemetry only in this mode

        // Converging helix: the circle tightens as height is used up, so the
        // spiral ends ON the target rather than leaving a horizontal offset of
        // one radius still to fly off at the bottom with no height left to
        // convert it (measured as a 49 m miss releasing from a fixed 40 m orbit).
        //
        // Floored at what the aircraft can actually turn, though: radius grows
        // as V^2, and a steep nose-down at idle builds speed fast, so a circle
        // tighter than that is not merely imprecise but unflyable -- the
        // aircraft departs the turn tangentially and leaves entirely. Measured
        // as a 125 m impact when this floor was absent and the demand fell to a
        // few metres. Holding a wide circle beats losing the target completely.
        const float V_orbit = MAX(state.ground_speed, AP_EMG_MIN_SPEED_MS);
        const float bank_lim = constrain_float(_roll_lim, 5.0f, 80.0f);
        const float min_flyable_r = 1.15f * sq(V_orbit) / (GRAVITY_MSS * tanf(radians(bank_lim)));
        out.orbit_radius_m = constrain_float(state.height_agl * _spiral_conv,
                                             min_flyable_r,
                                             MAX(_spiral_radius, min_flyable_r));

        if (range_to_target <= (out.orbit_radius_m * 2.0f) && _spiral_reached_ms == 0) {
            _spiral_reached_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EMG: over target, spiralling down");
        }

        // The helix runs all the way in rather than releasing into a separate
        // dive: by the bottom the radius has shrunk to a few metres, so there is
        // no run-in left to need.
        if (state.height_agl <= AP_EMG_IMPACT_ALT_M) {
            _phase = Phase::IMPACT;
            out.terminate = true;
            out.complete = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EMG: arrived %.0fm from target",
                          (double)range_to_target);
            break;
        }

        if ((now_ms - _phase_start_ms) > AP_EMG_DIVE_TIMEOUT_MS) {
            _phase = Phase::ABORTED;
            out.action = Action::NONE;
            out.complete = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EMG: could not reach orbit, aborting");
        }
        break;
    }

    case Phase::DESCEND: {
        out.action = Action::ATTITUDE;
        out.throttle = constrain_float(_dive_thr, 0.0f, 1.0f);

        const float height_above_target = state.height_agl;
        const float slant = norm(range_to_target, height_above_target);
        if (slant < _closest_slant_m) {
            _closest_slant_m = slant;
        }

        // Speed used to convert a turn rate into a bank angle. Airspeed is not
        // trustworthy on this airframe (ARSPD_USE=0 in the validated config),
        // so groundspeed is used, floored so the conversion cannot blow up.
        const float V = MAX(state.ground_speed, AP_EMG_MIN_SPEED_MS);

        // ---- lateral: proportional navigation + pursuit, summed as a TURN RATE
        // Bank proportional to bearing error produces PIO -- measured, the roll
        // response lags by seconds and the aircraft weaves. Working in turn-rate
        // space keeps the units consistent and lets PN supply the lead term.
        float los_rate = 0.0f;                       // rad/s
        if (_have_prev_los && dt > 1.0e-3f) {
            los_rate = radians(wrap_180(los_deg - _prev_los_deg)) / dt;
        }
        _prev_los_deg = los_deg;
        _have_prev_los = true;

        const float omega_pn = _nav_gain * los_rate;
        const float omega_pursuit = _pursuit_gain * radians(brg_err);
        float omega = omega_pn + omega_pursuit;
        const float omega_max = radians(_turn_rate);
        omega = constrain_float(omega, -omega_max, omega_max);

        float roll_deg = degrees(atan2f(omega * V, GRAVITY_MSS));
        roll_deg = constrain_float(roll_deg, -_roll_lim, _roll_lim);

        // ---- vertical: drive the FLIGHT-PATH angle onto the line-of-sight
        // elevation, so the velocity vector tracks the aim point rather than
        // just the nose.
        const float elev_to_tgt = degrees(atan2f(height_above_target,
                                                 MAX(range_to_target, 0.1f)));
        const float sin_gamma = constrain_float(state.climb_rate / V, -1.0f, 1.0f);
        const float gamma_now = degrees(asinf(sin_gamma));

        // Aggressive dive-in: for the first EMG_DIVE_TIME seconds, force the
        // target flight-path angle to EMG_DIVE_PITCH instead of the natural
        // line-of-sight elevation, so the nose goes down immediately rather
        // than steepening gradually as range closes. Same proportional
        // controller either way, so the handoff back to the line-of-sight
        // law is smooth, not a discontinuous jump. Left off inside the
        // terminal lock zone -- that phase needs the line-of-sight law.
        const float phase_elapsed_s = (now_ms - _phase_start_ms) * 0.001f;
        const bool dive_in_active = (_dive_time > 0.0f) &&
                                     (phase_elapsed_s < _dive_time) &&
                                     (slant >= _lock_dist);
        const float gamma_des = dive_in_active ? _dive_pitch : -elev_to_tgt;
        float pitch_deg = (state.pitch_cd * 0.01f) + _gamma_p * (gamma_des - gamma_now);
        pitch_deg = constrain_float(pitch_deg, _pitch_min, _pitch_max);

        // ---- terminal lock: inside lock_dist the line-of-sight rate goes
        // singular at the target and steering commands become garbage. Freeze
        // most of the bank and keep only the vertical channel live.
        if (slant < _lock_dist) {
            if (!_have_locked_roll) {
                _locked_roll_cd = roll_deg * 100.0f * 0.3f;
                _have_locked_roll = true;
            }
            out.roll_cd = _locked_roll_cd;
        } else {
            _have_locked_roll = false;
            out.roll_cd = roll_deg * 100.0f;
        }

        // ---- optional variable-angle body-rate dive (EMG_VDIVE_*): pitch is
        // commanded as a ROTATION RATE rather than an angle target, applied
        // via the rate controller directly (the same mechanism ACRO mode
        // uses), so it neither goes through the angle-based law's observed
        // ceiling nor (approaching +-90 deg) its Euler-angle singularity.
        // Tracks the SAME line-of-sight-derived target flight-path angle as
        // the angle-based law above (-elev_to_tgt) -- shallow while far out,
        // steepening as range closes, up to EMG_VDIVE_PITCH (the steepest
        // allowed) -- rather than a fixed dive angle, so the aircraft still
        // actually converges onto the target instead of diving straight
        // down wherever it happened to be when triggered. (A fixed target
        // was tried first and does not converge laterally at all -- there
        // is no altitude/time budget left for lateral correction once
        // diving near-vertically from an arbitrary trigger point.)
        // Proportional on the remaining flight-path-angle error, so it
        // self-regulates -- no separate "reached" state needed. Stays
        // rate-controlled all the way to impact: unlike the lateral/roll
        // law, the pitch geometry has no singularity approaching the
        // target (elev_to_tgt just smoothly approaches 90 deg as range
        // closes to zero), so there is no need to hand off to the
        // angle-based law near lock_dist the way roll does.
        if (_vdive_enable) {
            const float vdive_target_deg = MAX(-elev_to_tgt, (float)_vdive_pitch);
            out.action = Action::RATE_PITCH_ANGLE_ROLL;
            out.pitch_rate_dps = constrain_float(_vdive_rate_p * (vdive_target_deg - gamma_now),
                                                  -_vdive_max_rate, _vdive_max_rate);
        }
        out.pitch_cd = pitch_deg * 100.0f;

        // ---- arrival: the descent has reached the target's terrain ----
        if (height_above_target <= AP_EMG_IMPACT_ALT_M) {
            _phase = Phase::IMPACT;
            out.terminate = true;
            out.complete = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EMG: arrived %.0fm from target",
                          (double)range_to_target);
            break;
        }

        // ---- overshoot: range opening well past the closest approach ----
        if ((now_ms - _phase_start_ms) > AP_EMG_OVERSHOOT_GRACE_MS &&
            range_to_target > (_closest_slant_m + AP_EMG_OVERSHOOT_MARGIN_M) &&
            slant > 60.0f) {
            _phase = Phase::ABORTED;
            out.action = Action::NONE;
            out.complete = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "EMG: not closing (best %.0fm), aborting",
                          (double)_closest_slant_m);
            break;
        }

        if ((now_ms - _phase_start_ms) > AP_EMG_DIVE_TIMEOUT_MS) {
            _phase = Phase::ABORTED;
            out.action = Action::NONE;
            out.complete = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EMG: descent timed out, aborting");
            break;
        }
        break;
    }

    default:
        break;
    }

    _last_output = out;
    return out;
}

void AP_EmergencyDescent::abort()
{
    if (is_active()) {
        _phase = Phase::ABORTED;
    }
}

#endif  // AP_EMERGENCYDESCENT_ENABLED
