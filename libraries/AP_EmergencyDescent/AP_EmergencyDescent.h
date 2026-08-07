/// @file   AP_EmergencyDescent.h
/// @brief  Native emergency-descent guidance for ArduPlane AUTO missions.
///
/// This is a firmware port of the validated `emergency-desent.py` prototype.
/// It reproduces that tool's tested behaviour exactly: fly to an entry gate,
/// hold until the run-in to the target lines up, then a terminal descent under
/// proportional-navigation lateral guidance and flight-path-angle vertical
/// guidance onto the target, ending in impact. The guidance math lives here and
/// is deliberately free of any ArduPlane dependency; the vehicle glue feeds it
/// state and applies the commanded roll/pitch/throttle.
///
/// The four measured findings the prototype recorded and this port preserves:
///  - lateral bank is derived from a commanded TURN RATE via atan(omega*V/g),
///    never proportional to bearing error (that produced PIO);
///  - the dive is released only when heading error < EMG_ALIGN_TOL AND range is
///    still >= EMG_MIN_RNG_R * dive_distance (turning straight onto the target
///    spends the stand-off the descent needs);
///  - guidance runs at a fixed sub-rate so line-of-sight-rate differentiation is
///    not amplified by the full loop rate;
///  - vertical guidance drives the flight-path angle, not just the nose.

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

// If the run-in never lines up, descend anyway after this long: in a real
// emergency a slightly misaligned descent beats no descent.
#define AP_EMG_ALIGN_TIMEOUT_MS 90000U

// Speed floor for the turn-rate to bank conversion, so atan(omega*V/g) cannot
// misbehave at very low or momentarily-zero reported speed.
#define AP_EMG_MIN_SPEED_MS 5.0f

// Height above the target terrain at which the descent is considered to have
// arrived. The prototype used 0.5 m against a 10 Hz telemetry stream; in
// firmware the sample rate is the same but the airframe covers ~2 m per tick
// at descent speed, so a slightly larger band avoids skipping straight past it.
#define AP_EMG_IMPACT_ALT_M 1.0f

// Terminal guidance gives up if the range has been opening for this long past
// the closest approach. The grace period exists so a descent that begins
// mid-turn is not abandoned before the aircraft has swung onto the target.
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

    // Phase of the descent state machine. Mirrors the prototype's flow:
    // TRANSIT (fly to gate) -> ALIGN (hold, release on run-in) ->
    // DESCEND (terminal guidance) -> IMPACT (hand back to vehicle to disarm).
    enum class Phase : uint8_t {
        INACTIVE = 0,
        TRANSIT  = 1,
        ALIGN    = 2,
        DESCEND  = 3,
        IMPACT   = 4,
        COMPLETE = 5,
        ABORTED  = 6,
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
    // ArduPlane dependency: these three actions map directly onto in-tree
    // primitives (nav controller target, loiter, and direct nav_roll_cd /
    // nav_pitch_cd writes respectively).
    enum class Action : uint8_t {
        NONE = 0,       // do nothing; leave the flight path alone
        NAV_TO = 1,     // fly to nav_target using the normal nav controller
        LOITER = 2,     // loiter at nav_target
        ATTITUDE = 3,   // apply roll_cd / pitch_cd / throttle directly
    };

    struct Output {
        Action action = Action::NONE;
        Location nav_target;      // used by NAV_TO / LOITER
        float roll_cd = 0.0f;     // desired roll,  centidegrees (ATTITUDE)
        float pitch_cd = 0.0f;    // desired pitch, centidegrees (ATTITUDE)
        float throttle = 0.0f;    // 0..1                        (ATTITUDE)
        bool  terminate = false;  // true once IMPACT: vehicle should disarm
        bool  complete = false;   // descent finished (success or abort)
    };

    void init();

    // Begin a descent onto `target`, entering via `entry`. Captures the entry
    // altitude from `current`. Returns false (and does not activate) if the
    // geometry is invalid; reason is written to `reason` if non-null.
    bool start(const Location &entry, const Location &target,
               const Location &current, char *reason, uint8_t reason_len);

    // Run one guidance tick. Safe to call at loop rate; internally rate-limits
    // the guidance law to EMG_RATE_HZ. Advances the phase machine.
    Output update(const State &state);

    // Cancel immediately (pilot abort, mode change, new mission, etc.).
    void abort();

    bool is_active() const {
        return _phase == Phase::TRANSIT || _phase == Phase::ALIGN ||
               _phase == Phase::DESCEND;
    }
    Phase phase() const { return _phase; }
    const char *phase_name() const;

    // Derived geometry, valid after a successful start().
    float dive_distance() const { return _dive_distance_m; }
    float approach_bearing() const { return _approach_bearing_deg; }
    float run_in_bearing() const { return _run_in_bearing_deg; }

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
    AP_Float _lock_dist;      // slant range at which lateral authority is bled (m)
    AP_Float _entry_rad;      // entry-gate capture radius (m)
    AP_Float _align_tol;      // run-in heading tolerance for release (deg)
    AP_Float _min_rng_ratio;  // release requires range >= this * dive_distance
    AP_Float _dive_thr;       // throttle held through the descent (0..1)
    AP_Int8  _rate_hz;        // guidance sub-rate (Hz)
    AP_Float _min_alt;        // reject a descent commanded below this (m above home)

    // ---- state ----
    Phase _phase;
    Location _entry;
    Location _target;
    float _entry_alt_agl_m;       // captured entry altitude
    float _dive_distance_m;
    float _approach_bearing_deg;  // target -> entry
    float _run_in_bearing_deg;    // entry -> target

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
