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

    _phase = Phase::DESCEND;
    _phase_start_ms = AP_HAL::millis();
    _last_guidance_ms = 0;
    _have_prev_los = false;
    _closest_slant_m = FLT_MAX;
    _have_locked_roll = false;
    _last_output = Output{};

    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                  "EMG descent: diving onto target, %.0fm out, from %.0fm",
                  (double)_start_range_m, (double)_start_alt_agl_m);
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
        const float gamma_des = -elev_to_tgt;
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
