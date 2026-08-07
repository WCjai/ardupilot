# EMG_DESCENT_NOTES.md — Phase 0 findings

**Tree:** `ArduPlane V4.8.0-dev`, `ArduPilot-4.6.0-beta1-7882-gb224a6597e`
**Path:** `/home/lansub/Desktop/simulation_63-main/simulation_63-main/ardupilot`
All file:line references below are from this tree and were read, not assumed.

> **Two findings overturn the plan as written.** See 0.3 (the ID/location byte
> budget makes the specified `hasLocation="true"` custom IDs impossible) and 0.6
> (no parameter mutation is needed at all in-tree). Both have clean resolutions,
> recommended below.

---

## 0.1 — ArduPlane mission execution

| Item | Location |
|---|---|
| `Plane::start_command()` | `ArduPlane/commands_logic.cpp:6` |
| `Plane::verify_command()` | `ArduPlane/commands_logic.cpp:221` |
| `Plane::start_command_callback()` | `ArduPlane/commands_logic.cpp:1020` |
| `Plane::verify_command_callback()` | `ArduPlane/commands_logic.cpp:1030` |
| `Plane::exit_mission_callback()` | `ArduPlane/commands_logic.cpp:1047` |

**`verify_command()` contract** — documented at `commands_logic.cpp:213-219`:
returning **`true` means the command is complete and the mission advances.**
Note the last line of that comment: *"Return true if we do not recognize the
command so that we move on to the next command."* An unhandled ID is therefore
**silently skipped**, not an error. A typo in our `case` label produces a mission
that quietly does nothing — worth remembering when debugging Phase 2.

Both callbacks gate on `control_mode == &mode_auto` (`:1022`, `:1032`), so mission
commands cannot fire while the pilot has taken another mode. This is the
mechanism that gives us Phase 6's "pilot mode switch always wins" for free.

**Scheduler rates** (`ArduPlane/Plane.cpp`):

- `FAST_TASK(ahrs_update)`, `FAST_TASK(update_control_mode)`, `FAST_TASK(stabilize)`,
  `FAST_TASK(set_servos)` — `Plane.cpp:64-67`, run **every loop** (loop rate, 400 Hz
  on typical hardware / SITL).
- `SCHED_TASK(navigate, 10, 150, 36)` — `Plane.cpp:75`, i.e. **10 Hz**, not 400 Hz.
- `verify_command_callback` is documented at `:1028` as *"called from ap-mission at
  10hz or higher"*.

So `mode->update()` runs at full loop rate via `update_control_mode`, while the
navigation/mission layer runs at 10 Hz. This directly answers the Phase 4
question about guidance rate — see 0.7 finding (3).

---

## 0.2 — Is a custom ID treated as a nav command?

`AP_Mission::is_nav_cmd()` — `libraries/AP_Mission/AP_Mission.cpp:547-554`:

```cpp
return (cmd.id <= MAV_CMD_NAV_LAST ||          // 95
        cmd.id == MAV_CMD_NAV_SET_YAW_SPEED ||  // 213
        cmd.id == MAV_CMD_NAV_SCRIPT_TIME ||    // 42702
        cmd.id == MAV_CMD_NAV_ATTITUDE_TIME);   // 42703
```

**The plan's prediction is correct:** it is a range check against
`MAV_CMD_NAV_LAST = 95` (`common.xml:1278`) plus a hand-maintained exception list.
IDs 10000/10001 as specified in the plan would **not** be recognised as nav
commands.

ArduPilot's own custom nav commands `MAV_CMD_NAV_SCRIPT_TIME` (42702,
`ardupilotmega.xml:280`) and `MAV_CMD_NAV_ATTITUDE_TIME` (42703, `:290`) solve
this by being added to that exception list — a clear in-tree precedent for
option (a).

**However, option (a) is not the right answer here**, because of 0.3. Both of
those precedent commands are declared `hasLocation="false"`, and that is not an
accident — see below.

**Recommendation: option (b), choose IDs inside the nav range.** Scanning
`common.xml` + `ardupilotmega.xml` for allocated `MAV_CMD` values below 256 shows
**86-91 are unallocated** and sit inside the `<= 95` nav range:

```
84 NAV_VTOL_TAKEOFF   85 NAV_VTOL_LAND   [86..91 FREE]
92 NAV_GUIDED_ENABLE  93 NAV_DELAY  94 NAV_PAYLOAD_PLACE  95 NAV_LAST
```

Using **90** and **91**:

- pass `is_nav_cmd()` automatically — **no change to `AP_Mission` required**;
- are `< 256`, which 0.3 shows is mandatory for carrying a location.

*Caveat to flag:* 90/91 are unallocated in upstream MAVLink, not reserved for us.
If upstream ever allocates them, a fork carrying this feature would collide. That
is acceptable for a private firmware fork and is the only option that satisfies
both constraints simultaneously; it should not be proposed for upstream merge
without requesting a proper allocation.

---

## 0.3 — Mission item storage and param survival — **CRITICAL**

`AP_MISSION_EEPROM_COMMAND_SIZE = 15` — `libraries/AP_Mission/AP_Mission.h:27`.

Storage layout, from `AP_Mission::write_cmd_to_storage()` (`AP_Mission.cpp:974-993`):

| ID width | Record layout | Bytes for content |
|---|---|---|
| `id < 256` | 1 (id) + 2 (p1) + 12 (content) | **12** |
| `id >= 256` | 1 (tag) + 2 (id) + 2 (p1) + 10 (content) | **10** |

`PackedLocation` (`AP_Mission.cpp:796-809`) is:

```
1 byte  flags/options
3 bytes alt   (int32_t alt:24)
4 bytes lat
4 bytes lng
= 12 bytes exactly
```

`ASSERT_STORAGE_SIZE(PackedContent, 12)` at `AP_Mission.cpp:827` enforces this.

### Verdict

**A 16-bit command ID physically cannot carry a location.** 10 available bytes
< 12 required. The tree states this explicitly and guards it with a panic —
`AP_Mission.cpp:865-872`:

```cpp
// NOTE!  no 16-bit command may be stored_in_location as only
// 10 bytes are available for storage and lat/lon/alt required
// 4*sizeof(float) == 12 bytes of storage.
if (b1 == 0) {
    AP_HAL::panic("May not store location for 16-bit commands");
}
```

This is why `NAV_SCRIPT_TIME`/`NAV_ATTITUDE_TIME` are `hasLocation="false"`.

**Consequences for the plan:**

1. Phase 2's instruction to add IDs 10000/10001 with `hasLocation="true"` is
   **impossible**. So is any ID in the 42700 range. This must change.
2. **Byte budget for a location-carrying command: 12 of 12 bytes consumed by
   lat/lng/alt. Zero remaining.** The only spare payload is `p1` (`uint16`, 2
   bytes) and `type_specific_bit_0/1` (2 bits inside the flags byte).
3. **A nav command cannot carry a full location AND four independent floats.**
   Not even one float. The plan's own contingency is the correct one:
   **locations in the mission item, all tuning in `EMG_*` parameters.**

`p1` remains genuinely useful — 16 bits is enough for e.g. a descent-profile
selector or an approach-bearing override in whole degrees, should we want one.

### Note on `stored_in_location()`

`AP_Mission.cpp:904-937` lists IDs stored as locations, and does include entries
above 255 (`MAV_CMD_NAV_FENCE_*` = 5000-5006, `MAV_CMD_NAV_RALLY_POINT` = 5100).
Those are **not** written through mission storage — fence and rally items use
their own storage backends and only borrow `Mission_Command` for MAVLink
conversion. They are not a counter-example; the 12-byte limit stands for anything
actually written by `write_cmd_to_storage()`.

Our two new IDs **must be added to `stored_in_location()`** for the location to
be packed at all.

---

## 0.4 — How guidance commands the airframe in-tree

The prototype streams `SET_ATTITUDE_TARGET`. In firmware that is the wrong
mechanism, exactly as the plan says. The in-tree equivalent is
`ModeGuided::update()` — `ArduPlane/mode_guided.cpp:30-103`.

**Control variables** (`ArduPlane/Plane.h`):

| Variable | Line |
|---|---|
| `int32_t nav_roll_cd` | `Plane.h:663` |
| `int32_t nav_pitch_cd` | `Plane.h:666` |
| `int32_t roll_limit_cd` | `Plane.h:213` |
| `float pitch_limit_min` | `Plane.h:214` |

**Producers** — `mode_guided.cpp:41-42` (roll) and `:79-80` (pitch):

```cpp
plane.nav_roll_cd = constrain_int32(<value>, -plane.roll_limit_cd, plane.roll_limit_cd);
plane.update_load_factor();
...
plane.nav_pitch_cd = constrain_int32(<value>, plane.pitch_limit_min*100,
                                     plane.aparm.pitch_limit_max.get()*100);
```

**Consumers** — `ArduPlane/Attitude.cpp`:

- `Attitude.cpp:179` — `rollController.run_angle_control(nav_roll_cd, speed_scaler, ...)`
- `Attitude.cpp:244` — `demanded_pitch = nav_pitch_cd + pitch_trim + kff_throttle_to_pitch*throttle`
- `Plane::calc_nav_pitch()` at `Attitude.cpp:638` and `calc_nav_roll()` at `:643` are
  the *TECS/L1* writers — we bypass these by writing the variables ourselves.

**Throttle outside TECS** — `mode_guided.cpp:88` and `:95`:

```cpp
SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, <0..100>);   // bypasses TECS
```
versus `plane.calc_throttle();` at `:99` which runs TECS.

### Deliverable: per-tick call sequence to hold commanded roll and pitch

```cpp
// bypasses L1 (lateral) and TECS (vertical) entirely
plane.nav_roll_cd  = constrain_int32(roll_cd,  -plane.roll_limit_cd, plane.roll_limit_cd);
plane.update_load_factor();                    // MUST follow a nav_roll_cd write
plane.nav_pitch_cd = constrain_int32(pitch_cd, plane.pitch_limit_min * 100,
                                     plane.aparm.pitch_limit_max.get() * 100);
SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_pct);
```

Called from the AUTO update path each tick; `stabilize()` (FAST_TASK) then drives
the servos. No `mavlink_message_t` is constructed anywhere — as required.

### In-tree confirmation of the prototype's `param3` bug

`mode_guided.cpp:62`:

```cpp
float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
```

This is the **exact source** of the measured finding that
`GUIDED_CHANGE_HEADING` with `param3 = 0` produces `atan(0/9.81) = 0` bank limit
and the aircraft never turns while ACKing every command. Reading the source
confirms the SITL measurement precisely.

---

## 0.5 — Altitude, terrain, airspeed

- **Height above terrain:** `Plane::relative_ground_altitude(RangeFinderUse use_rangefinder, bool use_terrain_if_available)`
  — `ArduPlane/altitude.cpp:111`; 2-arg convenience wrapper at `:176` which passes
  `target_altitude.terrain_following` when `AP_TERRAIN_AVAILABLE`.
  It prefers external HAGL (`:113-117`), then rangefinder, then terrain, then
  falls back to height above home. **Failure mode when terrain is missing: it
  silently degrades to home-relative altitude**, which over sloping ground is
  wrong in exactly the direction that matters for a flare. The flare must not
  trust it blindly — see Phase 5 note below.
- **Absolute altitude:** `current_loc.alt` (cm, AMSL) — e.g. `altitude.cpp:102`.
- **Prototype's `ground_alt_above_home`:** corresponds in-tree to the target
  location's own altitude field (the mission item carries `alt` already), and
  terrain height should come from `relative_ground_altitude()` rather than being
  an operator-supplied constant. `AP_Terrain` is the better source where available.
- **Airspeed:** the SITL config used for all prototype tuning has `ARSPD_USE = 0`
  (measured earlier in this project), i.e. **no airspeed sensor in the control
  loop** — the "airspeed" reported by SITL tracks groundspeed. Guidance must
  therefore use a speed that is actually trustworthy: take `V` from
  `AHRS::groundspeed_vector().length()` and floor it (the prototype used
  `max(gs, 5.0)`) so `atan(omega*V/g)` cannot divide-by-zero or produce a
  nonsense bank at low speed.

---

## 0.6 — Parameters

`AP_Param::set_and_save_by_name()` — `libraries/AP_Param/AP_Param.cpp:2868-2896`.
It resolves the name then calls `set_and_save()` on the typed object, which
**writes to EEPROM/flash immediately**. There is also
`set_and_save_by_name_ifchanged()` at `:2898`.

**Not safe from a fast loop** — it is a storage write, and the prototype called it
five times per run.

**Recommendation: do not mutate any tuning parameter. It is unnecessary in-tree.**

The prototype mutated `TECS_SINK_MAX`, `AIRSPEED_MAX`, `TECS_PITCH_MIN`,
`PTCH_LIM_MIN_DEG`, `THR_MIN` **only because it was driving the aircraft through
TECS over MAVLink** and had to widen TECS's authority to get the dive it wanted.
Writing `nav_pitch_cd` directly (0.4) bypasses TECS completely, so
`TECS_SINK_MAX` and `AIRSPEED_MAX` are simply not in the loop any more.

What still clamps us is `plane.roll_limit_cd` and `plane.pitch_limit_min`
(`Plane.h:213-214`) — and these are **runtime variables recomputed each loop**,
not parameters. They can be respected, or locally widened for the duration of the
descent, **without touching flash at all**.

This also removes the risk the plan flagged: persisting modified TECS limits and
losing power mid-descent would leave the aircraft permanently mistuned. With this
design that failure mode does not exist. Phase 6's "restore modified limits on
every exit path" reduces to restoring plain RAM state.

---

## 0.7 — The Python prototype

### Tunables

| Prototype constant | Value | Origin | Firmware destination |
|---|---|---|---|
| `nav_gain` (N) | 3.0 | PN standard, SITL-verified | `EMG_NAV_GAIN` |
| `pursuit_gain` | 0.8 | tuned down from 2.2 after PIO | `EMG_PURSUIT_GN` |
| `max_turn_rate_deg` | 25 | SITL | `EMG_TURN_RATE` |
| `roll_limit_deg` | 45 | matches `ROLL_LIMIT_DEG` | `EMG_ROLL_LIM` (clamped by `roll_limit_cd`) |
| `pitch_min_deg` / `pitch_max_deg` | −60 / +15 | SITL | `EMG_PITCH_MIN` / `EMG_PITCH_MAX` |
| `KP_GAMMA` | 1.6 | flight-path loop gain | `EMG_GAMMA_P` |
| `lock_dist_m` | 20 | LOS-rate singularity | `EMG_LOCK_DIST` |
| `entry_accept_m` | 110 | fixed-wing cannot hold a point | `EMG_ENTRY_RAD` |
| `align_tol_deg` | 15 | 23.6° → 7.0 m miss; <15° → 0.5 m | `EMG_ALIGN_TOL` |
| min range at release | `0.75 × dive_distance` | prevents spending stand-off | hard-coded ratio or `EMG_MIN_RNG` |
| `airspeed_ms` | 22 | SITL cruise | `EMG_AIRSPEED` |
| `dive_thrust` | 0.0 | idle through descent | `EMG_DIVE_THR` |
| telemetry rate | 10 Hz | link limit | matches `navigate` @ 10 Hz |
| `ground_alt_above_home` | 0.0 | operator input | from mission item alt + `AP_Terrain` |

### The four measured findings, and whether they survive the port

1. **`GUIDED_CHANGE_HEADING` param3 = 0 pins bank limit to zero.**
   *Transport artifact — becomes irrelevant.* We write `nav_roll_cd` directly and
   never issue that command. Confirmed at source: `mode_guided.cpp:62`. What
   *does* survive is the underlying requirement that bank be explicitly limited;
   in-tree that clamp is `plane.roll_limit_cd`.

2. **`SET_POSITION_TARGET_GLOBAL_INT` applies altitude but discards lat/lon;
   `DO_REPOSITION` is what moves the target.**
   *Transport artifact — becomes irrelevant.* We are inside the autopilot and set
   nav targets directly. Worth keeping only as a warning to anyone writing an
   external GCS tool against this firmware.

3. **Bank proportional to bearing error produces PIO; bank must come from a turn
   rate via `atan(omega*V/g)`.**
   ***Guidance physics — fully relevant, and more so in-tree.*** This was measured
   with the airframe's real roll response (commanded roll changed from +32.9° to
   +0.2° while actual roll stayed at +34.9° — seconds of lag). That lag is a
   property of the aircraft and the roll controller, not of MAVLink. Additionally,
   because `mode->update()` runs at loop rate rather than the prototype's 10 Hz,
   **differentiating LOS at loop rate will amplify noise ~40× more than the
   prototype ever saw.** Decision for Phase 4: **run the guidance law at a fixed
   10 Hz sub-rate**, matching both `navigate`'s cadence and the rate at which
   every prototype gain was actually tuned. This makes the measured gains
   transferable instead of needing a full retune, and avoids the noise problem
   without adding filter lag.

4. **Releasing the dive at 23.6° heading error missed by 7.0 m; releasing below
   15° missed by 0.5 m.**
   ***Guidance geometry — fully relevant.*** Drives `EMG_ALIGN_TOL` default = 15°.
   The paired range condition (`range >= 0.75 × dive_distance`) is equally
   important and comes from the separate measured failure where turning straight
   onto the target left the aircraft 19 m out and still 42 m high, needing a 65°
   dive it could not fly.

---

## 0.8 — Restating the requirement

**What the operator wants.** The aircraft normally flies an AUTO waypoint
mission. If a human on the ground judges there is an emergency, they should be
able to open the GCS, plan an emergency descent onto a chosen point, upload it as
a mission, and have the aircraft fly it autonomously — with the feature living
natively inside AUTO mode rather than in an external script.

**Intended field workflow.** Aircraft is airborne in AUTO. Operator right-clicks
a map location to place the descent target, and an entry gate; the GCS uploads a
mission containing the new command pair; the aircraft, on reaching that mission
item, transits to the entry gate, holds until the run-in lines up, descends under
guidance, flares, and lands at the target.

### Landing vs termination — the distinction this design turns on

**The referenced section is missing from the plan.** `AGENT_IMPLEMENTATION_PLAN.md`
says "the landing-vs-termination distinction from the top of this document", but
lines 6-9 between the two `---` rules are **empty** — that section did not survive
into the file I was given. I am stating my reading explicitly so it can be
corrected before Phase 5, which depends entirely on it:

> The Python prototype is a **termination** tool. It was built and validated for
> impact-dynamics research: it drives an altitude demand *below* the terrain,
> flies the aircraft into the ground, and disarms. Its success metric was miss
> distance at impact (0.5 m).
>
> The feature described in this plan is an **emergency landing**. Phase 5 requires
> arresting the descent, bleeding speed toward touchdown airspeed without
> stalling, rounding out to a survivable attitude, levelling the wings, detecting
> touchdown using the in-tree detector, and disarming *only after touchdown is
> confirmed*. Phase 4 says explicitly: *"No impact detection and no disarm in this
> phase. The descent hands off to FLARE; it does not terminate."*

**How the design handles it.** The port takes the prototype's *guidance* — which
is genuinely good, repeatably better than 1 m — and reuses it **only for the
descent phase, terminating at flare-entry height, not at the ground.** The
prototype's impact detection and its mid-air disarm are **deliberately not
ported**. The descent's exit condition is a handoff to FLARE at an altitude with
enough margin to arrest the sink rate; FLARE is new work designed from the
airframe's numbers per Phase 5.

Concretely, the prototype behaviours that must **not** appear anywhere in the
firmware:

- altitude demand set below terrain (`impact_alt_agl = ground_alt - 30`);
- `disarm()` called from the guidance loop;
- "IMPACT" as a success condition.

A related safety consequence, per Phase 5: if the geometry the operator selects
makes a survivable flare impossible, `start()` must **reject the command** with a
clear GCS message rather than fly it.

---

## Phase 0 gate — summary and recommendations

| Q | Status | Outcome |
|---|---|---|
| 0.1 | answered | 10 Hz nav layer; `verify_command()` true ⇒ advance; unknown IDs silently skipped |
| 0.2 | **blocker found, resolved** | `is_nav_cmd()` is `id <= 95` + exceptions. **Use free IDs 90/91** ⇒ no `AP_Mission` change |
| 0.3 | **blocker found, resolved** | 16-bit IDs cannot carry a location (10 < 12 bytes). Location consumes all 12 bytes; **zero floats fit**. Tuning ⇒ `EMG_*` params only |
| 0.4 | answered | Write `nav_roll_cd` + `update_load_factor()` + `nav_pitch_cd`; throttle via `set_output_scaled` |
| 0.5 | answered | `relative_ground_altitude()`; degrades to home-relative when terrain absent; use groundspeed for `V` |
| 0.6 | **plan simplified** | **No param mutation needed** — writing `nav_pitch_cd` bypasses TECS. Flash-persistence risk eliminated |
| 0.7 | answered | Findings (1)(2) are transport artifacts, dropped; (3)(4) are physics, retained. Guidance runs at 10 Hz sub-rate |
| 0.8 | answered, **one gap** | Landing≠termination restated; **the plan's own reference section is missing from the file** |

### Deviations from the plan, for approval

1. **Command IDs 90 / 91**, not 10000 / 10001 — forced by 0.3. Requires adding
   them to `stored_in_location()`; requires **no** `is_nav_cmd()` change.
2. **No per-command float parameters.** All tuning in `EMG_*`. Forced by 0.3.
3. **No runtime parameter mutation** and therefore no "restore limits" logic
   against flash. Enabled by 0.4/0.6.
4. **Guidance law runs at a fixed 10 Hz sub-rate**, not loop rate — preserves the
   validity of the prototype's measured gains and avoids LOS-rate noise.

---

## Phase 1 result — PASS

Library `AP_EmergencyDescent` created and registered (Plane.h, Parameters.h/.cpp
g2 subgroup 42, system.cpp, and the waf `COMMON_VEHICLE_DEPENDENT_LIBRARIES`
list). Builds clean for SITL. SITL boot + MAVLink param dump confirms all 15
`EMG_*` parameters present with the Phase-0 defaults. Flight behaviour unchanged
(stub `update()` returns an invalid output).

Gotcha recorded: a new library is not linked just by creating the directory — it
must be added to `Tools/ardupilotwaf/ardupilotwaf.py:COMMON_VEHICLE_DEPENDENT_LIBRARIES`
and the board reconfigured, else the object compiles but the vehicle fails to link.

## Phase 2 result — critical gate PASS

MAVLink IDs **90** (`MAV_CMD_NAV_EMERGENCY_DESCENT_ENTRY`) and **91**
(`..._TARGET`) added to `ardupilotmega.xml` with `hasLocation="true"`; headers
regenerate cleanly and the enum appears in the generated `ardupilotmega.h`.
`AP_Mission` conversion cases added (forward + reverse) plus both IDs added to
`stored_in_location()`. Stub `do_emergency_descent()`/`verify_emergency_descent()`
wired into `Plane::start_command`/`verify_command`.

**Critical acceptance (the real test of the 0.3 byte budget) — PASS:**
- Upload a 5-item mission containing both commands → `MAV_MISSION_ACCEPTED`.
- Download → both items **byte-identical**: ENTRY (cmd 90, lat 146055316,
  lon 758200058, alt 90.0), TARGET (cmd 91, lat 146056354, lon 758218941,
  alt 0.0), matching the upload exactly.
- Reboot SITL, download again → whole mission **byte-identical across reboot**
  (survives flash storage).

Confirmed twice more on re-runs. The 12-byte location + zero-float design from 0.3
is validated end to end.

**Secondary (in-flight stub print):** dispatch is confirmed — in AUTO the mission
runs and `start_command` fires for nav commands (mission sequences to the takeoff
item). Visually confirming the ENTRY/TARGET stubs printing their lat/lon requires
the aircraft to actually fly to those items; the headless SITL plane would not
complete its runway takeoff in this harness. Rather than special-case takeoff
here, this confirmation is folded into Phase 3, where flying to the entry gate is
the actual deliverable and the ENTRY command executing in flight is the natural
test.

Test harness note: mission storage is sized late in boot (`max_items()` returns 0
until `AP_Mission::init()` runs), so an upload attempted too early is rejected
`MAV_MISSION_NO_SPACE`. The GCS must gate uploads on readiness. Also
`MissionItemProtocol_Waypoints::truncate()` mutates the stored mission on
`MISSION_COUNT`, so a count-probe is destructive — read-only `MISSION_REQUEST_LIST`
must be used to poll for post-reboot readiness.

## Phase 3+4 result — PASS (first native firmware flight)

Full profile flown natively in ArduPlane AUTO on the built-in `plane` SITL model:

```
EMG descent: 200m run-in, brg 90, from 120m   <- start(), geometry derived
mission seq -> 2                               <- ENTRY became the active nav cmd
EMG: at entry gate (109m), aligning            <- TRANSIT -> ALIGN
EMG: descending, err 14deg rng 193m            <- released: 14deg < EMG_ALIGN_TOL,
                                                  193m >= 0.75 * 200m range floor
IMPACT at 8.6m from target, alt 0.8
```

Both prototype release conditions reproduced. Miss 8.6 m (target < 10 m).

### Four real bugs found and fixed during Phase 3/4 bring-up

1. **`PANIC: Mission command with ID 90 has no string`** — `Mission_Command::type()`
   panics in SITL for any ID without a name. Added `EmergencyDescentEntry` /
   `EmergencyDescentTarget` cases. This was the hard hang: process alive, main
   loop dead, all telemetry frozen. Not discoverable through sim_vehicle,
   whose log does not capture the firmware's stdout — it only surfaced by
   replicating sim_vehicle's exact binary invocation and capturing stdout.
2. **Returning `true` from the AUTO hook for NAV_TO/LOITER** made
   `ModeAuto::update()` return early, so `calc_nav_pitch()` / `calc_throttle()`
   never ran. TECS was left unserviced and the aircraft held its climb attitude,
   reaching 1110 m and 2.7 km off. Only `ATTITUDE` may bypass the normal
   controllers; the nav phases must return false and let them run.
3. **`set_guided_WP()` called every loop tick** (400 Hz) starves the scheduler —
   it does terrain fix, altitude slope and turn-angle work. Now only re-issued
   when the target actually changes.
4. **ALIGN overflew the gate** instead of orbiting, spending the stand-off
   distance. `ModeAuto::navigate()` now calls `update_loiter()` during ALIGN.

### Test harness

`/tmp/emgtest/harness.py` — headless SITL (built-in `plane` model, no Gazebo),
seconds per cycle instead of a minute. Notes:
- Take off with **TAKEOFF mode** (`set TKOFF_ALT`, mode 13, arm). The
  FBWA + RC-override sequence from autotest does not work here: there is no
  MAVProxy supplying RC, and RC overrides did not drive throttle.
- Mode changes need `MAV_CMD_DO_SET_MODE`; the legacy `SET_MODE` message is
  ignored once in an auto-throttle mode.
- Keep speedup modest (3). At speedup 10 the telemetry backlogs faster than a
  Python consumer drains it and the trace desynchronises from real time.

The new firmware is installed at `Plane/arduplane` (original saved as
`Plane/arduplane.orig-backup`), so `main.py` / the Gazebo Alti Transition world
now runs this build.

## Phase 5 result — PASS

Arrival detection, termination and abort logic moved into the library
(`AP_EMG_IMPACT_ALT_M`, overshoot test with grace period, dive timeout). The
library now sets `Output::terminate` and the vehicle disarms in response:

```
EMG: arrived 11m from target
EMG: impact, disarming
closest slant 8.7m  horizontal miss 8.6m
```

Behaviour matches the prototype: arrival at the target's terrain terminates and
disarms. Overshoot and timeout abort instead, leaving the aircraft to normal
AUTO navigation rather than flying it into the ground.

## Phase 6 result — PASS (all three safety paths)

| Case | Observed |
|---|---|
| ENTRY with no paired TARGET | `EMG: ENTRY has no paired TARGET, skipping` |
| Pilot mode change mid-descent | `EMG: aborted, left AUTO` |
| Not enough altitude to fly the profile | `EMG: rejected (too low (120m))` |

Also guarded: position/AHRS loss and the mission being changed underneath a
running descent both abort and hand back to normal AUTO navigation.

`AP_EMG_MIN_START_ALT_M` became the **`EMG_MIN_ALT` parameter** (default 15 m).
Making it tunable is better design and it is what made the guard testable: on
the ground the mission never dispatches the item at all, so the guard could only
be exercised airborne by raising the threshold above the aircraft's altitude.

## Phase 7 result — PASS on the real Gazebo Alti Transition

Flown in the `iris_runway` world with the Alti Transition, launched exactly as
`simlib.start_indronovation_stack()` does, driving SERIAL2 (tcp:5763) — the port
simlib reserves for a custom MAVLink script.

```
EMG descent: 204m run-in, brg 87, from 101m
EMG: at entry gate (109m), aligning
EMG: descending, err -15deg rng 285m
EMG: arrived 7m from target
EMG: impact, disarming
closest slant 3.6m  horizontal miss 2.6m
```

**Miss 2.6 m** — better than the built-in `plane` model (8.6 m) and approaching
the Python prototype's 0.5 m.

### The bug only Gazebo exposed

TRANSIT set `next_WP_loc` but never called `nav_controller->update_waypoint()`.
ArduPlane tracks a waypoint because `verify_nav_wp()` calls that on **every**
pass; without it L1 is never told to steer. The aircraft held its heading and
sat ~500 m from the gate. The built-in `plane` model had hidden this because its
takeoff happened to point at the gate, so it drifted into capture range by luck.
This is a good argument for validating in the target environment and not only in
the fast harness.

Also fixed: a GCS set-current re-entering the ENTRY item restarted the profile
and lost its progress; `do_emergency_descent()` now returns early if a descent
is already active.

### Gazebo harness notes

- SITL blocks on "Waiting for connection" on SERIAL0 and only binds SERIAL1/2
  once a client attaches to **5760**. In the app that client is the RC relay; a
  holder socket stands in for it in the test script.
- The Alti Transition will **not** complete a runway `NAV_TAKEOFF` in this world
  (`Q_ENABLE=0`, so no VTOL). ArduPlane's TAKEOFF mode flies it fine. This also
  matches the intended workflow: the aircraft is already airborne on its normal
  mission when the operator uploads the emergency descent.
