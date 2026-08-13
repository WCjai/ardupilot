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
/// Optional descending spiral (EMG_SPIRAL_*, default disabled). The dive below
/// converts altitude into DISTANCE: it needs run-in room to bend its flight
/// path onto the line of sight, and given a close target from high up it
/// cannot (the angle that hits from 400 m up with the target 50 m out is
/// 83 deg, unreachable within 50 m of travel), so it overflies and circles
/// back. SPIRAL converts altitude into a TURN instead, corkscrewing down over
/// the target: nose down at EMG_SPIRAL_PTCH while banked, radius tightening
/// with remaining height (EMG_SPIRAL_CONV) so the helix ends on the target
/// rather than leaving an offset to fly off at the bottom.
///
/// KEEP EMG_SPIRAL_PTCH SHALLOW. A steep spiral is self-contradictory and
/// degenerates into a plain steep dive that wanders off the target. Turning
/// needs the lift vector to have a horizontal component; past roughly 45 deg
/// nose-down the aircraft is close enough to vertical that lift points
/// sideways rather than centripetally, so bank rotates it about an axis aimed
/// at the ground instead of curving its path. Measured directly: commanding
/// -55 deg (achieving -70 deg), the orbit radius was varied from 3 m to over
/// 73 m and the trajectory did not change at all -- same impact point to
/// within 0.1 m, because the orbit guidance had no authority over the path.
/// Compounding it, with no drag device altitude must become speed, and turn
/// radius grows as V^2, so a steep spiral is also the fastest way to make the
/// circle unflyable. The radius is floored at what is achievable for the
/// current speed and bank limit, but that floor cannot rescue a steep command.
///
/// So the two modes trade cleanly and neither dominates:
///   - plain dive (SPIRAL off): sub-metre accuracy, but converts altitude into
///     distance, so with a close target from high up it overflies and circles
///     back before it can line up;
///   - shallow spiral: holds station over the target and does not overfly, but
///     descends slowly, because a shallow flight path is what leaves lift
///     available to turn with.
/// Fast, steep, and localised together needs a drag device (flaps/spoilers);
/// with one, the steep descent no longer runs away in speed and the trade
/// dissolves. Absent that, prefer the plain dive with adequate run-in distance.
///
/// Optional variable-angle body-rate dive (EMG_VDIVE_*, default disabled):
/// the ANGLE-based law above commands a target pitch and converges to it
/// through the vehicle's normal fixed-wing attitude controller -- which,
/// like all Euler-angle attitude control, gets unreliable approaching +-90
/// deg (roll and yaw become coupled/degenerate there), and was found
/// empirically to have an unexplained ceiling well short of that regardless.
/// EMG_VDIVE instead commands a pitch ROTATION RATE
/// (Action::RATE_PITCH_ANGLE_ROLL, applied via the vehicle's rate controller
/// directly -- the same mechanism ACRO mode uses), which never asks for an
/// angle to converge to and so has neither problem. Critically, the RATE
/// law's target is not a fixed dive angle -- it tracks the same line-of-
/// sight-derived flight-path angle the angle-based law above does (shallow
/// far out, steepening as range closes), capped at EMG_VDIVE_PITCH as the
/// steepest allowed. A fixed target was tried first and does not converge
/// laterally onto the target at all: diving near-vertically leaves no
/// altitude/time budget for lateral correction, so it just impacts near
/// wherever it was triggered. Proportional on the remaining flight-path-
/// angle error (rate = Kp * (target - current gamma), bounded by
/// EMG_VDIVE_RMAX), so it self-regulates: large rate while far from the
/// target, tapering to zero as it's reached, no separate "reached" state
/// needed. Roll is unaffected -- still the ordinary angle-based bank from
/// the PN law above. Pitch stays rate-controlled all the way to impact
/// (unlike roll, the pitch geometry has no singularity approaching the
/// target -- elev_to_tgt just smoothly approaches 90 deg as range closes to
/// zero -- so there is no need to hand off to the angle-based law near
/// lock_dist the way roll does).

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
    // SPIRAL is numbered last rather than placed before DESCEND (where it sits
    // in the flow) so the existing values stay stable for anything comparing
    // or logging them.
    enum class Phase : uint8_t {
        INACTIVE = 0,
        DESCEND  = 1,
        IMPACT   = 2,
        COMPLETE = 3,
        ABORTED  = 4,
        SPIRAL   = 5,
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
        ORBIT = 3,                  // orbit nav_target at orbit_radius_m; roll from the vehicle's
                                    // own loiter guidance, pitch/throttle from here
    };

    struct Output {
        Action action = Action::NONE;
        float roll_cd = 0.0f;        // desired roll,  centidegrees (ATTITUDE, RATE_PITCH_ANGLE_ROLL)
        float pitch_cd = 0.0f;       // desired pitch, centidegrees (ATTITUDE, ORBIT)
        float pitch_rate_dps = 0.0f; // desired pitch rotation rate, deg/s, +up (RATE_PITCH_ANGLE_ROLL)
        float throttle = 0.0f;       // 0..1                        (ATTITUDE, RATE_PITCH_ANGLE_ROLL, ORBIT)
        Location nav_target;         // point to orbit                              (ORBIT)
        float orbit_radius_m = 0.0f; // orbit radius, m                             (ORBIT)
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

    bool is_active() const { return _phase == Phase::DESCEND || _phase == Phase::SPIRAL; }
    Phase phase() const { return _phase; }
    const char *phase_name() const;

    // Cheap accessors onto the cached last output, for the vehicle glue's
    // attitude-control path to check between guidance ticks (it runs at loop
    // rate; this library's own guidance is sub-rate-limited) without
    // re-running the guidance law or rebuilding a State.
    // ORBIT is included: the spiral's steep pitch must go through the rate
    // controller for the same reason the dive does -- driven as an angle target
    // it stalls around -20 deg, which starves the helix of descent rate.
    bool wants_rate_pitch() const
    {
        return _last_output.action == Action::RATE_PITCH_ANGLE_ROLL ||
               _last_output.action == Action::ORBIT;
    }
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
    AP_Int8  _spiral_enable;  // orbit overhead to get into position before diving
    AP_Float _spiral_radius;  // orbit radius while positioning (m)
    AP_Float _spiral_pitch;   // pitch held during the positioning orbit (deg)
    AP_Float _spiral_conv;    // orbit radius per metre of remaining height (tightens the helix)
    AP_Int8  _vdive_enable;   // enable the variable-angle body-rate dive
    AP_Float _vdive_pitch;    // steepest flight-path angle the body-rate dive may command (deg, e.g. -85)
    AP_Float _vdive_rate_p;   // gain: commanded rate (deg/s) per degree of flight-path-angle error
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
    uint32_t _spiral_reached_ms;  // when the positioning orbit was first reached; 0 until then
    float _prev_los_deg;
    uint32_t _prev_los_ms;
    bool  _have_prev_los;
    float _closest_slant_m;       // best (smallest) slant range seen in DESCEND
    float _locked_roll_cd;        // frozen roll command inside the lock zone
    bool  _have_locked_roll;
    Output _last_output;          // held between guidance ticks
};

#endif  // AP_EMERGENCYDESCENT_ENABLED
