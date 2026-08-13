/// @file   AP_EmergencyDescent.h
/// @brief  Native emergency-descent guidance for ArduPlane AUTO missions.
///
/// Single-point behaviour: given only a target location, the aircraft dives
/// directly onto it from wherever it currently is, under proportional-
/// navigation lateral guidance and flight-path-angle vertical guidance,
/// ending in impact. There is no separate entry gate and no alignment/orbit
/// phase -- an earlier revision flew to an entry gate and held in a loiter
/// until the run-in bearing lined up before diving. That loiter is what a
/// fixed-wing does to correct heading, so on release the aircraft could be
/// established on an arbitrary point of the circle, not travelling through
/// the gate itself: observed in testing as flying a wide circle near the
/// entry point and passing the intended line. Diving straight from the
/// current position removes that failure mode entirely -- proportional
/// navigation is exactly the control law for converging onto a target from
/// an arbitrary starting position and heading, which is what it is for.
///
/// The guidance math lives here and is deliberately free of any ArduPlane
/// dependency; the vehicle glue feeds it state and applies the commanded
/// roll/pitch/throttle.
///
/// Findings measured in the original prototype and preserved here:
///  - lateral bank is derived from a commanded TURN RATE via atan(omega*V/g),
///    never proportional to bearing error (that produced PIO);
///  - guidance runs at a fixed sub-rate so line-of-sight-rate differentiation
///    is not amplified by the full loop rate;
///  - vertical guidance drives the flight-path angle, not just the nose.
///
/// Optional aggressive dive-in (EMG_DIVE_PITCH / EMG_DIVE_TIME, default
/// disabled): for the first EMG_DIVE_TIME seconds of the descent the vertical
/// law's target flight-path angle is forced to EMG_DIVE_PITCH instead of the
/// line-of-sight elevation to the target, so the aircraft pitches down hard
/// immediately rather than gradually steepening as range closes. It hands off
/// to the ordinary line-of-sight law smoothly (same proportional controller,
/// only its target changes) once the window elapses or the terminal lock
/// distance is reached, whichever comes first.
///
/// Optional near-vertical dive (EMG_VDIVE_*, default disabled): the ANGLE-based
/// law above commands a target pitch and converges to it through the
/// vehicle's normal fixed-wing attitude controller -- which, like all Euler-
/// angle attitude control, gets unreliable approaching +-90 deg (roll and yaw
/// become coupled/degenerate there), and was found empirically to have an
/// unexplained ceiling well short of that regardless. EMG_VDIVE instead
/// commands a pitch ROTATION RATE (Action::RATE_PITCH_ANGLE_ROLL, applied via
/// the vehicle's rate controller directly -- the same mechanism ACRO mode
/// uses), which never asks for an angle to converge to and so has neither
/// problem. It is a proportional-on-angle-error rate command (rate = Kp *
/// (EMG_VDIVE_PITCH - current pitch), bounded by EMG_VDIVE_MAX_RATE), so it
/// self-regulates: large rate while far from the target, tapering to zero as
/// it's reached, no separate "reached" state needed. Roll is unaffected --
/// still the ordinary angle-based bank from the PN law above. Active outside
/// the terminal lock distance; inside it, hands off to the angle-based law
/// for final aim-point precision.

#pragma once

#include "AP_EmergencyDescent_config.h"

#if AP_EMERGENCYDESCENT_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

// Minimum height above home at which a descent may be started. Below this
// there is no room to fly the profile at all.
#define AP_EMG_MIN_START_ALT_M 15.0f

// Speed floor for the turn-rate to bank conversion, so atan(omega*V/g) cannot
// misbehave at very low or momentarily-zero reported speed.
#define AP_EMG_MIN_SPEED_MS 5.0f

// Height above the target terrain at which the descent is considered to have
// arrived. The prototype used 0.5 m against a 10 Hz telemetry stream; in
// firmware the sample rate is the same but the airframe covers ~2 m per tick
// at descent speed, so a slightly larger band avoids skipping straight past it.
#define AP_EMG_IMPACT_ALT_M 1.0f

// Terminal guidance gives up if the range has been opening for this long past
// the closest approach. The grace period exists so a descent that begins on
// an arbitrary heading is not abandoned before the aircraft has swung onto
// the target.
#define AP_EMG_OVERSHOOT_GRACE_MS 12000U
#define AP_EMG_OVERSHOOT_MARGIN_M 40.0f

// Absolute cap on the descent phase.
#define AP_EMG_DIVE_TIMEOUT_MS 120000U

class AP_EmergencyDescent {
public:
    AP_EmergencyDescent();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_EmergencyDescent);

    static AP_EmergencyDescent *get_singleton() { return _singleton; }

    // Phase of the descent state machine: DESCEND (terminal guidance) ->
    // IMPACT (hand back to vehicle to disarm).
    enum class Phase : uint8_t {
        INACTIVE = 0,
        DESCEND  = 1,
        IMPACT   = 2,
        COMPLETE = 3,
        ABORTED  = 4,
    };

    // Snapshot of vehicle state fed in each guidance tick, in SI/AP units.
    struct State {
        Location current;        // current position
        float ground_speed;      // m/s (used for V; airspeed is unreliable, see notes)
        float climb_rate;        // m/s, +up
        float height_agl;        // m above target terrain
        float roll_cd;           // current roll,  centidegrees
        float pitch_cd;          // current pitch, centidegrees
        float heading_deg;       // current heading (yaw), degrees 0..360
    };

    // What the vehicle should do this tick. Keeps the library free of any
    // ArduPlane dependency: ATTITUDE maps directly onto direct nav_roll_cd /
    // nav_pitch_cd writes.
    enum class Action : uint8_t {
        NONE = 0,                  // do nothing; leave the flight path alone
        ATTITUDE = 1,               // apply roll_cd / pitch_cd / throttle directly
        RATE_PITCH_ANGLE_ROLL = 2,  // roll_cd as an angle target; pitch_rate_dps as a body rate
    };

    struct Output {
        Action action = Action::NONE;
        float roll_cd = 0.0f;        // desired roll,  centidegrees (ATTITUDE, RATE_PITCH_ANGLE_ROLL)
        float pitch_cd = 0.0f;       // desired pitch, centidegrees (ATTITUDE)
        float pitch_rate_dps = 0.0f; // desired pitch rotation rate, deg/s, +up (RATE_PITCH_ANGLE_ROLL)
        float throttle = 0.0f;       // 0..1                        (ATTITUDE, RATE_PITCH_ANGLE_ROLL)
        bool  terminate = false;     // true once IMPACT: vehicle should disarm
        bool  complete = false;      // descent finished (success or abort)
    };

    void init();

    // Begin a descent straight onto `target` from wherever the aircraft
    // currently is -- no entry gate, no alignment phase. Captures the start
    // altitude from `current`. Returns false (and does not activate) if the
    // geometry is invalid; reason is written to `reason` if non-null.
    bool start(const Location &target, const Location &current,
               char *reason, uint8_t reason_len);

    // Run one guidance tick. Safe to call at loop rate; internally rate-limits
    // the guidance law to EMG_RATE_HZ. Advances the phase machine.
    Output update(const State &state);

    // Cancel immediately (pilot abort, mode change, new mission, etc.).
    void abort();

    bool is_active() const { return _phase == Phase::DESCEND; }
    Phase phase() const { return _phase; }
    const char *phase_name() const;

    // Cheap accessors onto the cached last output, for the vehicle glue's
    // attitude-control path to check between guidance ticks (it runs at loop
    // rate; this library's own guidance is sub-rate-limited) without
    // re-running the guidance law or rebuilding a State.
    bool wants_rate_pitch() const { return _last_output.action == Action::RATE_PITCH_ANGLE_ROLL; }
    float pitch_rate_dps() const { return _last_output.pitch_rate_dps; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_EmergencyDescent *_singleton;

    // ---- parameters (EMG_*) ----
    AP_Int8  _enable;
    AP_Float _airspeed;       // target airspeed / cruise speed (m/s)
    AP_Float _nav_gain;       // proportional-navigation constant N
    AP_Float _pursuit_gain;   // pursuit term on bearing error (1/s)
    AP_Float _turn_rate;      // max commanded turn rate (deg/s)
    AP_Float _roll_lim;       // max bank in the descent (deg)
    AP_Float _pitch_min;      // min (most nose-down) pitch (deg)
    AP_Float _pitch_max;      // max (nose-up) pitch (deg)
    AP_Float _gamma_p;        // flight-path-angle loop gain
    AP_Float _dive_pitch;     // forced nose-down pitch target for the initial dive-in (deg)
    AP_Float _dive_time;      // duration to force _dive_pitch before the LOS law takes over (s); 0 disables
    AP_Int8  _vdive_enable;   // enable the near-vertical body-rate dive
    AP_Float _vdive_pitch;    // target pitch for the body-rate dive (deg, e.g. -85)
    AP_Float _vdive_rate_p;   // gain: commanded rate (deg/s) per degree of pitch error
    AP_Float _vdive_max_rate; // max commanded pitch rate (deg/s)
    AP_Float _lock_dist;      // slant range at which lateral authority is bled (m)
    AP_Float _dive_thr;       // throttle held through the descent (0..1)
    AP_Int8  _rate_hz;        // guidance sub-rate (Hz)
    AP_Float _min_alt;        // reject a descent commanded below this (m above home)

    // ---- state ----
    Phase _phase;
    Location _target;
    float _start_alt_agl_m;       // captured altitude at start()
    float _start_range_m;         // distance to target at start() (informational)

    // guidance-loop memory
    uint32_t _last_guidance_ms;   // rate limiter for the guidance law
    uint32_t _phase_start_ms;
    float _prev_los_deg;
    uint32_t _prev_los_ms;
    bool  _have_prev_los;
    float _closest_slant_m;       // best (smallest) slant range seen in DESCEND
    float _locked_roll_cd;        // frozen roll command inside the lock zone
    bool  _have_locked_roll;
    Output _last_output;          // held between guidance ticks
};

#endif  // AP_EMERGENCYDESCENT_ENABLED
