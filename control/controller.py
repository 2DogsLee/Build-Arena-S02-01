"""S02 mission controller (v1) — payload + reusable booster, LONE ORB.

Session-owned implementation. Constants calibrated from this session's cal01
flight telemetry (see knowledge.md / design_budget.md). Offline design in
offline/ascent_pwm.py and offline/descent_sim3.py.

Frame conventions (from telemetry, Unity-like): world x=East, y=Up, z=North.
The build frame of the authored machine is recovered as M = R_root_live * R0^-1,
so build +Z (machine up / thrust axis) maps to the world thrust direction.
"""
from __future__ import annotations

import json
import math
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from controller_sdk import ControllerClient, targets_by_guid
from controller_sdk.protocol import ENV_MACHINE_BSG, ENV_RUN_DIR

CENTER = np.array([0.0, -600.0, 0.0])
MU = 8_456_800.0
R_SURFACE = 600.0
G_MAX = 22.0

F_FULL = 2030.0
FLOW_FULL = 7.69
FUEL_PER_UNIT = 0.035
DRY_TOTAL = 18.15          # v4: +1.0 (4 extra reaction wheels, 8 total)
PAYLOAD_MASS = 1.5
BOOSTER_DRY = DRY_TOTAL - PAYLOAD_MASS
FUEL_CAP = 200.0           # v2: two 100-unit tanks linked by a Fuel Line
TANK_CAP_MIN = 50.0        # blocks with fuel_capacity >= this count as main tanks
DRAG_C0 = 0.028
DRAG_H = 90.0

PWM_MIN_PULSE = 0.08

ASC = dict(r_target=1420.0, h_start=40.0, h_end=600.0, k_vr=2.5, k_vt=4.0,
           vr_max=40.0, rp_target=1320.0)

V_AFTER_BURN = 15.0
R_APO_TARGET = 2100.0    # predicted ballistic apogee radius for the vertical boost
PITCH_RATE = 1.2         # rad/s attitude slew at apogee (vacuum)
INS_ETA = 0.06           # insertion burn elevation bias (rad above horizon)
INS_VFRAC = 0.995        # fraction of circular speed for the insertion cutoff
H_BRAKE = 700.0
H_HOLD = 250.0          # hover altitude above the pad for the translate phase
DEORBIT_BIAS = 0.25      # rad of range bias: ballistic impact aimed this much past the pad
V_BRAKE = 32.0
HOVER_H = 60.0
H_LAND = 80.0
V_SINK = 0.6

KP_ATT = 1.6
KD_ATT = 1.6
K_WHEEL = 0.16
MOTOR_CAP = 0.48
WHEEL_RATE_UNIT = 1200.0 * math.pi / 180.0


def unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros(3)


def wrap_pi(a):
    return ((a + math.pi) % (2 * math.pi)) - math.pi


def gravity_vec(pos):
    rel = pos - CENTER
    r = float(np.linalg.norm(rel))
    n = rel / r
    return -min(G_MAX, MU / (r * r)) * n, r, n


def elements(pos, vel):
    rel = pos - CENTER
    r = float(np.linalg.norm(rel))
    v2 = float(vel @ vel)
    eps = v2 / 2 - MU / r
    hh = float(np.linalg.norm(np.cross(rel, vel)))
    if eps >= 0:
        return dict(a=None, e=1.0, rp=hh * hh / MU / 2, ra=None, period=None, eps=eps)
    a = -MU / (2 * eps)
    e = math.sqrt(max(0.0, 1 + 2 * eps * hh * hh / MU ** 2))
    rp = hh * hh / (MU * (1 + e))
    ra = hh * hh / (MU * (1 - e)) if e < 1 - 1e-9 else None
    return dict(a=a, e=e, rp=rp, ra=ra, eps=eps,
                period=2 * math.pi * math.sqrt(a ** 3 / MU) if a > 0 else None)


def predict_impact(pos, vel, dt=0.06, max_t=400.0, c0=DRAG_C0, H=DRAG_H):
    p = np.array(pos, dtype=float)
    v = np.array(vel, dtype=float)
    t = 0.0
    while t < max_t:
        r = float(np.linalg.norm(p))
        n = p / r
        g = min(G_MAX, MU / (r * r))
        sp = float(np.linalg.norm(v))
        a = -g * n
        if sp > 1e-9 and r > R_SURFACE:
            c = c0 * math.exp(-max(r - R_SURFACE, 0.0) / H)
            a = a - c * sp * v
        pm = p + v * dt / 2
        vm = v + a * dt / 2
        rm = float(np.linalg.norm(pm))
        nm = pm / rm
        gm = min(G_MAX, MU / (rm * rm))
        spm = float(np.linalg.norm(vm))
        a2 = -gm * nm
        if spm > 1e-9:
            c = c0 * math.exp(-max(rm - R_SURFACE, 0.0) / H)
            a2 = a2 - c * spm * vm
        p = p + vm * dt
        v = v + a2 * dt
        t += dt
        if float(np.linalg.norm(p)) <= R_SURFACE:
            return dict(hit=True, pos=p, ang=math.atan2(p[1], p[0]), v=v, t=t)
    return dict(hit=False)


class Roles:
    pass


def main():
    run_dir = Path(os.environ[ENV_RUN_DIR])
    bsg_path = Path(os.environ[ENV_MACHINE_BSG])

    client = ControllerClient.from_environment(poll_interval=0.002)
    # The mod data dir holds >90k files from earlier sessions; the SDK's
    # gc_acked_actions() globs it on every telemetry read (~300 ms). Disable it
    # during the run; the runner clears action files when the run ends.
    client.gc_acked_actions = lambda sequence_applied: 0
    client.arm()
    frame0 = client.wait_until_running(timeout=120)
    # the MOD may not have written block_table.json yet right after a sandbox
    # load; retry instead of crashing (v2m15 died on this race)
    for _attempt in range(120):
        try:
            client.load_block_table()
            break
        except Exception as exc:
            print(f"waiting for block table ({exc})", flush=True)
            time.sleep(0.5)
    else:
        raise RuntimeError("block table never became available")

    # --------------------------------------------------------- roles
    roles = Roles()
    roles.engines, roles.wheels, roles.decouplers = [], [], []
    for ch in client.channels:
        g = ch.block_guid.lower()
        if ch.name == "ThrustKey":
            roles.engines.append(g)
        elif ch.name == "ExplodeKey":
            roles.decouplers.append(g)
        elif ch.name in ("LeftKey", "RightKey") and g not in roles.wheels:
            roles.wheels.append(g)
    roles.key_index = {}
    roles.slider_index = {}
    for ch in client.channels:
        roles.key_index[(ch.block_guid.lower(), ch.name)] = ch.index
    for sl in client.sliders:
        roles.slider_index[(sl.block_guid.lower(), sl.name)] = sl.index

    saved, id_of = {}, {}
    for b in ET.parse(bsg_path).getroot().findall("Blocks/Block"):
        g = b.get("guid").lower()
        pos = np.array([float(b.find("Transform/Position").get(k)) for k in ("x", "y", "z")])
        rot = R.from_quat([float(b.find("Transform/Rotation").get(k))
                           for k in ("x", "y", "z", "w")])
        saved[g] = (pos, rot)
        id_of[g] = int(b.get("id"))
    inv = {v: k for k, v in id_of.items()}
    roles.root = inv.get(0)
    roles.tanks = sorted([g for g, i in id_of.items() if i == 99])
    roles.decoupler = inv.get(4)
    roles.payload_blocks = sorted([g for g, i in id_of.items() if i == 63])
    axis_build = {g: unit(saved[g][1].apply([0.0, 0.0, 1.0])) for g in roles.wheels}

    f = frame0
    d0t = targets_by_guid(f, block_guids=client._block_guids)
    root0 = np.array(d0t[roles.root]["position"])
    up0 = unit(root0 - CENTER)
    R0 = R.from_quat(list(d0t[roles.root]["rotation"]))
    cand = np.array([0.0, 0.0, 1.0])
    d0h = unit(cand - (cand @ up0) * up0)
    p0 = unit(np.cross(up0, d0h))
    ground_alt = float(np.linalg.norm(root0 - CENTER) - R_SURFACE)

    def state_of(sample):
        d = targets_by_guid(sample, block_guids=client._block_guids)
        if roles.root not in d:
            raise RuntimeError("root block missing from telemetry")
        root = d[roles.root]
        pos = np.array(root["position"])
        vel = np.array(root["velocity"])
        omega = np.array(root["angular_velocity"])
        rot = R.from_quat(list(root["rotation"]))
        M = rot * R0.inv()
        thrust_dir = unit(M.apply([0.0, 1.0, 0.0]))  # build Up is +Y in BSG coords
        gv, r, n = gravity_vec(pos)
        vr = float(vel @ n)
        vt_vec = vel - vr * n
        vt = float(np.linalg.norm(vt_vec))
        tank_fuel = None
        tank_parts = []
        for tg in roles.tanks:          # both Fuel Barrel Big blocks (id 99)
            if tg in d and d[tg].get("fuel") is not None:
                tank_parts.append(float(np.atleast_1d(d[tg]["fuel"])[0]))
        if tank_parts:
            tank_fuel = float(sum(tank_parts))
        payload = None
        best = None
        for g in roles.payload_blocks:
            if g in d:
                p = np.array(d[g]["position"])
                vv = np.array(d[g]["velocity"])
                dist = float(np.linalg.norm(p - pos))
                if best is None or dist > best[0]:
                    best = (dist, g, p, vv)
        if best is not None:
            payload = dict(guid=best[1], pos=best[2], vel=best[3], dist=best[0])
        wheel_axis = {g: unit(M.apply(axis_build[g])) for g in roles.wheels}
        wheel_rate = {}
        for g in roles.wheels:
            if g in d:
                rel = np.array(d[g]["angular_velocity"]) - omega
                wheel_rate[g] = float(rel @ wheel_axis[g])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(thrust_dir @ n)))))
        return dict(t=float(sample.simulation_time), pos=pos, vel=vel, omega=omega,
                    M=M, thrust_dir=thrust_dir, gv=gv, r=r, n=n,
                    h=r - R_SURFACE, h_rel=r - R_SURFACE - ground_alt,
                    vr=vr, vt=vt, vt_vec=vt_vec, speed=float(np.linalg.norm(vel)),
                    fuel=tank_fuel, tilt_deg=tilt,
                    integrity=(None if sample.machine.machine_integrity is None
                               else float(sample.machine.machine_integrity)),
                    alive=(None if sample.machine.alive_block_count is None
                           else int(sample.machine.alive_block_count)),
                    payload=payload, wheel_axis=wheel_axis, wheel_rate=wheel_rate)

    # --------------------------------------------------------- actuation
    def engine_records(throttle):
        sl, keys = [], []
        if throttle > 0.5:
            for g in roles.engines:
                sl.append((roles.slider_index[(g, "fthrust")], 2.0))
                keys.append((roles.key_index[(g, "ThrustKey")], 1.0))
        else:
            for g in roles.engines:
                sl.append((roles.slider_index[(g, "fthrust")], 0.0))
        return sl, keys

    def wheel_records(s, alpha):
        sl, keys = [], []
        if wheel_release:
            for g in roles.wheels:
                sl.append((roles.slider_index[(g, "speed")], 0.0))
            return sl, keys
        for g in roles.wheels:
            a = s["wheel_axis"][g]
            motor_now = s["wheel_rate"].get(g, 0.0) / WHEEL_RATE_UNIT
            torque_axis = float(alpha @ a)
            target = max(-MOTOR_CAP, min(MOTOR_CAP, motor_now - K_WHEEL * torque_axis))
            if abs(target) < 0.012:
                target = 0.0
            sl.append((roles.slider_index[(g, "speed")], abs(target)))
            if (g, "damper") in roles.slider_index:
                sl.append((roles.slider_index[(g, "damper")], 0.0))
            if abs(target) > 0.004:
                key = "LeftKey" if target > 0 else "RightKey"
                if (g, key) in roles.key_index:
                    keys.append((roles.key_index[(g, key)], 1.0))
        return sl, keys

    def att_alpha(s, desired, kp, kd):
        cur = s["thrust_dir"]
        err = np.cross(cur, desired)
        omega = np.array(s["omega"])
        perp = omega - float(omega @ cur) * cur
        return kp * err - kd * perp

    log, events = [], []
    phase = "pad"
    st0 = state_of(f)
    t0 = st0["t"]
    launch_pos = st0["pos"].copy()
    def plane_angle(p):
        dv = p - CENTER
        return math.atan2(float(dv @ d0h), float(dv @ up0))

    pad_ang = plane_angle(launch_pos)
    pwm_acc, fire_timer = PWM_MIN_PULSE, 0.0
    pitch_t0 = 0.0
    deorbit_t0 = None
    last_dpred = -1e9
    deorbit_err = 0.0
    last_h = 1e9
    wheel_release = False
    last_move_t = 0.0
    deorbit_v0 = 0.0
    prev_t = 0.0
    sep_time = None
    payload_orbit = None
    sweep_deg = 0.0
    payload_rmin = 1e12
    rmin_qualified = True
    prev_payload_dir = None
    last_pred_t = -1e9
    pred_now = None
    prev_geom_err = None
    prev_geom_err2 = None
    landed_time = None
    rest_since = None
    last_records = None
    decouple_until = -1.0
    abort = ""
    last_loop = time.monotonic()
    loop_acc, loop_n = 0.0, 0
    final_root_pos = None
    final_payload_pos = None

    def send(records):
        nonlocal last_records
        if records != last_records:
            client.send_indexes(records)
            last_records = records

    try:
        while True:
            f = client.next_sample(timeout=5.0)
            s = state_of(f)
            t = s["t"] - t0
            now = time.monotonic()
            loop_acc += now - last_loop
            loop_n += 1
            last_loop = now

            fuel = s["fuel"] if s["fuel"] is not None else 0.0
            fuel_mass = FUEL_PER_UNIT * fuel
            mass = (BOOSTER_DRY if sep_time is not None else DRY_TOTAL) + fuel_mass
            gv, r, n = s["gv"], s["r"], s["n"]
            vr, vt, h, h_rel = s["vr"], s["vt"], s["h"], s["h_rel"]
            h_hat = unit(np.cross(p0, n))
            cur_ang_now = plane_angle(s["pos"])
            d_ang = (pad_ang - cur_ang_now) % (2 * math.pi)
            herr_signed = wrap_pi(pad_ang - cur_ang_now) * r
            herr = herr_signed
            vh = float(s["vel"] @ h_hat)

            # payload tracking from separation onward
            if s["payload"] is not None and sep_time is not None:
                ppos = s["payload"]["pos"]
                pdir = unit(ppos - CENTER)
                if prev_payload_dir is not None:
                    c = max(-1.0, min(1.0, float(prev_payload_dir @ pdir)))
                    sweep_deg += math.degrees(math.acos(c))
                prev_payload_dir = pdir
                pr = float(np.linalg.norm(ppos - CENTER))
                payload_rmin = min(payload_rmin, pr)
                if pr <= 1100.0:
                    rmin_qualified = False

            desired = s["thrust_dir"]
            wheel_release = False
            throttle = 0.0
            alpha = np.zeros(3)
            decouple_records = []

            if s["integrity"] is not None and s["integrity"] < 0.90:
                abort = "integrity_below_0.90"

            if phase == "pad":
                alpha = att_alpha(s, desired, 1.0, 2.0)
                if t > 0.8:
                    phase = "ascent"
                    events.append(dict(t=round(t, 2), event="ignition",
                                       fuel=round(fuel, 2), tilt=round(s["tilt_deg"], 2)))
            elif phase == "ascent":
                # ---- closed-loop lofted ascent -------------------------------
                # 1) vertical boost while the *predicted* ballistic apogee is
                #    below R_APO_TARGET (the body stays tail-first, so the
                #    windvane torque cannot fight the wheels).
                # 2) coast to apogee, pitch over to horizontal in vacuum.
                # 3) horizontal insertion burn until the orbit is circular-ish.
                denom = 1.0 / r - (vr * vr) / (2.0 * MU)
                r_apo_pred = (1.0 / denom) if denom > 1e-12 else 1e12
                desired = n
                if r_apo_pred < R_APO_TARGET:
                    throttle = 1.0
                else:
                    throttle = 0.0
                    phase = "coast_up"
                    events.append(dict(t=round(t, 2), event="boost_end",
                                       h=round(h, 1), vr=round(vr, 1),
                                       r_apo_pred=round(r_apo_pred, 0),
                                       fuel=round(fuel, 2)))
                alpha = att_alpha(s, desired, 1.6, 1.4)
                if t > 60:
                    abort = "ascent_timeout"
            elif phase == "coast_up":
                desired = n
                alpha = att_alpha(s, desired, 1.4, 1.6)
                throttle = 0.0
                if vr <= 0.5:
                    phase = "pitch"
                    pitch_t0 = t
                    events.append(dict(t=round(t, 2), event="apogee",
                                       h=round(h, 1), vr=round(vr, 1),
                                       vt=round(vt, 1), fuel=round(fuel, 2)))
            elif phase == "pitch":
                throttle = 0.0
                phi = min(math.pi / 2.0, PITCH_RATE * (t - pitch_t0))
                desired = unit(n * math.cos(phi) + h_hat * math.sin(phi))
                alpha = att_alpha(s, desired, 1.6, 1.6)
                if phi >= math.pi / 2.0 - 1e-6 and \
                        float(s["thrust_dir"] @ h_hat) > 0.93:
                    phase = "ins_burn"
                    events.append(dict(t=round(t, 2), event="ins_burn_start",
                                       h=round(h, 1), vt=round(vt, 1),
                                       tilt=round(s["tilt_deg"], 1),
                                       fuel=round(fuel, 2)))
            elif phase == "ins_burn":
                desired = unit(h_hat + INS_ETA * n)
                alpha = att_alpha(s, desired, 1.8, 1.4)
                v_circ = math.sqrt(MU / r)
                throttle = 1.0 if vt < 0.99 * v_circ * INS_VFRAC else 0.0
                if float(s["thrust_dir"] @ desired) < 0.70:
                    throttle = 0.0
                el = elements(s["pos"], s["vel"])
                if (vt >= 0.99 * v_circ * INS_VFRAC or
                        (el["rp"] is not None and el["rp"] >= ASC["rp_target"]
                         and abs(vr) < 20)) and h > 300:
                    phase = "coast"
                    events.append(dict(t=round(t, 2), event="cutoff",
                                       h=round(h, 1),
                                       rp=(round(el["rp"], 1) if el["rp"] else None),
                                       ra=(round(el["ra"], 1) if el["ra"] else None),
                                       speed=round(s["speed"], 1), vt=round(vt, 1),
                                       fuel=round(fuel, 2)))
                if t > 90:
                    abort = "ascent_timeout"
            elif phase == "coast":
                desired = n
                alpha = att_alpha(s, desired, 0.9, 1.8)
                el = elements(s["pos"], s["vel"])
                if t - (0.0) > 1.0 and el["rp"] is not None and el["rp"] > 1180:
                    phase = "separate"
                    sep_time = t
                    decouple_until = t + 0.4
                    events.append(dict(t=round(t, 2), event="decouple_cmd",
                                       rp=round(el["rp"], 1), h=round(h, 1),
                                       fuel=round(fuel, 2)))
            elif phase in ("separate", "orbit", "deorbit", "fall"):
                el = elements(s["pos"], s["vel"])
                if phase == "separate":
                    desired = n
                    alpha = att_alpha(s, desired, 0.9, 1.8)
                    dist = s["payload"]["dist"] if s["payload"] else 0.0
                    if (s["payload"] is not None and dist > 4.0) or t - sep_time > 3.0:
                        elp = elements(s["payload"]["pos"], s["payload"]["vel"])
                        payload_orbit = elp
                        prev_payload_dir = unit(s["payload"]["pos"] - CENTER)
                        phase = "orbit"
                        events.append(dict(t=round(t, 2), event="separated",
                                           dist=round(dist, 2),
                                           payload_rp=round(elp["rp"], 1),
                                           payload_ra=(round(elp["ra"], 1)
                                                       if elp["ra"] else None),
                                           fuel=round(fuel, 2)))
                elif phase == "orbit":
                    desired = n
                    alpha = att_alpha(s, desired, 0.9, 1.8)
                    if t - last_pred_t > 0.4 and vt > 1e-3 and t - sep_time > 2.0:
                        last_pred_t = t
                        vmag = float(np.linalg.norm(s["vel"]))
                        dv = max(0.0, vmag - V_AFTER_BURN)
                        if dv > 5.0:
                            burn_dir = -unit(s["vel"])
                            p = predict_impact(s["pos"], s["vel"] + burn_dir * dv)
                            if p["hit"]:
                                pred_now = p
                                lead = (plane_angle(p["pos"]) - cur_ang_now) % (2 * math.pi)
                                err = wrap_pi(d_ang - lead)
                                crossed = (prev_geom_err is not None
                                           and prev_geom_err > 0.0 >= err)
                                prev_geom_err = err
                                if abs(err) < 0.04 or crossed:
                                    phase = "deorbit"
                                    events.append(dict(t=round(t, 2),
                                                       event="deorbit_start",
                                                       dv=round(dv, 1), h=round(h, 1),
                                                       err_deg=round(math.degrees(err), 2),
                                                       align=round(float(s["thrust_dir"] @ (-unit(s["vel"]))), 3),
                                                       fuel=round(fuel, 2)))
                                elif abs(err) < 0.60:
                                    # pre-align retrograde before the burn window
                                    desired = -unit(s["vel"])
                                    alpha = att_alpha(s, desired, 1.8, 1.6)
                    if t - (sep_time or 0) > 500:
                        abort = "orbit_wait_timeout"
                elif phase == "deorbit":
                    desired = -unit(s["vel"])
                    alpha = att_alpha(s, desired, 2.2, 1.5)
                    if deorbit_t0 is None:
                        deorbit_t0 = t
                        deorbit_v0 = s["speed"]
                        deorbit_err = 0.0
                    # burn only when roughly retrograde
                    if (t - deorbit_t0 > 2.5
                            and float(s["thrust_dir"] @ desired) > 0.55):
                        throttle = 1.0
                    done = False
                    if t - last_dpred > 0.35:
                        last_dpred = t
                        pr = predict_impact(s["pos"], s["vel"])
                        if pr["hit"]:
                            lead = (plane_angle(pr["pos"]) - cur_ang_now) % (2 * math.pi)
                            deorbit_err = wrap_pi(d_ang - lead)
                            # err < 0 => impact still lands beyond the pad
                            done = deorbit_err >= 0.0
                        else:
                            done = False
                    if done or t - deorbit_t0 > 18.0:
                        phase = "fall"
                        events.append(dict(t=round(t, 2), event="deorbit_done",
                                           speed=round(s["speed"], 1),
                                           dv=round(deorbit_v0 - s["speed"], 1),
                                           dt=round(t - deorbit_t0, 2),
                                           err_deg=round(math.degrees(deorbit_err), 2),
                                           fuel=round(fuel, 2)))
                elif phase == "fall":
                    desired = n
                    alpha = att_alpha(s, desired, 1.6, 1.6)
                    if h <= H_BRAKE:
                        phase = "brake"
                        events.append(dict(t=round(t, 2), event="brake_start",
                                           h=round(h, 1), speed=round(s["speed"], 1),
                                           tilt=round(s["tilt_deg"], 1),
                                           fuel=round(fuel, 2)))
            elif phase == "brake":
                desired = -unit(s["vel"])
                alpha = att_alpha(s, desired, 2.0, 1.5)
                if float(s["thrust_dir"] @ desired) > 0.75:
                    throttle = 1.0
                if s["speed"] <= V_BRAKE:
                    phase = "land"
                    events.append(dict(t=round(t, 2), event="land_start",
                                       h=round(h, 1), speed=round(s["speed"], 1),
                                       herr=round(herr, 1), fuel=round(fuel, 2)))
            elif phase == "approach":
                # hover at ~250 above the pad and null the horizontal error with
                # a bounded tilt; only then descend.  The deorbit ballistic
                # prediction carries a ~100-250 unit drag-model bias, which this
                # translation phase removes (the land law alone has too short a
                # horizon to null it).
                r_hold = R_SURFACE + ground_alt + H_HOLD
                vr_ref = max(-12.0, min(12.0, 0.30 * (r_hold - r)))
                a_up = float(-(gv @ n)) + 3.5 * (vr_ref - vr)
                a_up = max(0.2 * float(-(gv @ n)),
                           min(0.95 * F_FULL / mass, a_up))
                pad_pt = unit(launch_pos - CENTER) * r
                e_h = pad_pt - s["pos"]
                e_h = e_h - float(e_h @ n) * n
                e_norm = float(np.linalg.norm(e_h))
                if e_norm > 1000.0:
                    e_h = e_h * (1000.0 / e_norm)
                v_h_vec = s["vel"] - vr * n
                a_hv = 0.8 * e_h - 2.5 * v_h_vec
                a_hn = float(np.linalg.norm(a_hv))
                tmax = math.tan(math.radians(35.0))
                if a_hn > tmax * a_up and a_hn > 1e-9:
                    a_hv = a_hv * (tmax * a_up / a_hn)
                avec = n * a_up + a_hv
                amag = float(np.linalg.norm(avec))
                desired = avec / amag if amag > 1e-9 else n
                throttle = min(1.0, amag / (F_FULL / mass))
                # low-gain, rate-dominant attitude in the land phase: the
                # high-gain loop limit-cycles at ~2.4 s / +/-30 deg (actuator
                # + loop delay), which is what puts the body over at touchdown
                alpha = att_alpha(s, desired, 0.6, 2.5)
                if (e_norm < 18.0 and float(np.linalg.norm(v_h_vec)) < 3.0) or h_rel < 140.0:
                    phase = "land"
                    events.append(dict(t=round(t, 2), event="land_start",
                                       h=round(h, 1), speed=round(s["speed"], 1),
                                       herr=round(herr, 1), fuel=round(fuel, 2)))
            elif phase == "land":
                if True:
                    if h_rel > 200.0:
                        vr_ref = -max(V_SINK, min(15.0, 0.20 * h_rel))
                    else:
                        vr_ref = -max(0.8, 0.04 * max(h_rel, 0.0))
                    g_here = float(-(gv @ n))
                    a_up = g_here + 2.0 * (vr_ref - vr)
                    if vr < -32.0:
                        a_up = 0.95 * F_FULL / mass     # hard speed-safety brake
                    a_up = max(0.2 * g_here, min(0.95 * F_FULL / mass, a_up))
                    pad_pt = unit(launch_pos - CENTER) * r
                    e_h = pad_pt - s["pos"]
                    e_h = e_h - float(e_h @ n) * n
                    e_norm = float(np.linalg.norm(e_h))
                    if e_norm > 600.0:
                        e_h = e_h * (600.0 / e_norm)
                    v_h_vec = s["vel"] - vr * n
                    if h_rel < 20.0:
                        v_des = np.zeros(3)
                    else:
                        v_des = 0.28 * e_h
                        v_dn = float(np.linalg.norm(v_des))
                        v_cap = 12.0 if e_norm > 300.0 else 8.0
                        if v_dn > v_cap:
                            v_des = v_des * (v_cap / v_dn)
                    a_hv = 1.2 * (v_des - v_h_vec)
                    a_hn = float(np.linalg.norm(a_hv))
                    tmax = math.tan(math.radians(14.0)) * max(0.60, min(1.0, h_rel / 120.0))
                    lat_cap = tmax * a_up
                    if a_hn > lat_cap and a_hn > 1e-9:
                        a_hv = a_hv * (lat_cap / a_hn)
                    avec = n * a_up + a_hv
                    amag = float(np.linalg.norm(avec))
                    desired = avec / amag if amag > 1e-9 else n
                    throttle = min(1.0, amag / (F_FULL / mass))
                    if h_rel < 3.0 and abs(vr) < 3.0:
                        throttle = 0.0
                    if abs(h - last_h) > 0.04:
                        last_move_t = t
                    if t - last_move_t > 0.7 or (h_rel < 0.4 and s["speed"] < 0.8):
                        phase = "wait"
                        landed_time = t
                        final_root_pos = s["pos"].copy()
                        events.append(dict(t=round(t, 2), event="touchdown",
                                           h=round(h_rel, 2),
                                           speed=round(s["speed"], 2),
                                           herr=round(herr, 2), fuel=round(fuel, 2)))
                alpha = att_alpha(s, desired, 2.0, 1.6)
            elif phase in ("wait", "done"):
                alpha = np.zeros(3)
                wheel_release = True
                if s["speed"] < 0.35:
                    if rest_since is None:
                        rest_since = t
                else:
                    rest_since = None
                if (sweep_deg >= 1080.0 and rmin_qualified and rest_since is not None
                        and t - rest_since >= 10.0):
                    phase = "done"
                    events.append(dict(t=round(t, 2), event="mission_complete",
                                       sweep_deg=round(sweep_deg, 1),
                                       payload_rmin=round(payload_rmin, 1)))
                    break

            if decouple_until > t:
                for g in roles.decouplers:
                    decouple_records.append((roles.key_index[(g, "ExplodeKey")], 1.0))

            if abort:
                events.append(dict(t=round(t, 2), event="abort", reason=abort))
                break
            if fuel <= 1.0 and phase not in ("wait", "done", "land"):
                abort = "fuel_low"
                events.append(dict(t=round(t, 2), event="abort", reason=abort))
                break
            if t > 1800:
                abort = "mission_timeout"
                events.append(dict(t=round(t, 2), event="abort", reason=abort))
                break

            last_h = h
            prev_t = t
            # ---- global PWM: engines only run at full thrust; the controller
            # duty is realized as full-thrust pulses (a partial fthrust setting
            # is ~3x less fuel-efficient and non-linear, see cal01).
            duty_cmd = max(0.0, min(1.0, throttle))
            if duty_cmd >= 0.985:
                thr_out = 1.0
            elif duty_cmd <= 1e-3:
                thr_out = 0.0
                pwm_acc = min(pwm_acc, PWM_MIN_PULSE)
            else:
                if fire_timer > 1e-9:
                    fire_timer -= 0.04
                    thr_out = 1.0
                else:
                    pwm_acc += duty_cmd * 0.04
                    if pwm_acc >= PWM_MIN_PULSE - 1e-9:
                        pwm_acc -= PWM_MIN_PULSE
                        fire_timer = max(PWM_MIN_PULSE - 0.04, 0.0)
                        thr_out = 1.0
                    else:
                        thr_out = 0.0
            sl_e, key_e = engine_records(thr_out)
            sl_w, key_w = wheel_records(s, alpha)
            records = sl_e + sl_w + key_e + key_w + decouple_records
            send(records)
            if s["payload"] is not None:
                final_payload_pos = s["payload"]["pos"].copy()
            log.append(dict(t=round(t, 3), phase=phase, h=round(h, 2),
                            h_rel=round(h_rel, 2), vr=round(vr, 2), vt=round(vt, 2),
                            speed=round(s["speed"], 2),
                            fuel=(None if s["fuel"] is None else round(s["fuel"], 2)),
                            thr=round(throttle, 2), tilt=round(s["tilt_deg"], 2),
                            herr=round(herr, 2), sweep=round(sweep_deg, 2),
                            p_rmin=(round(payload_rmin, 1) if payload_rmin < 1e11 else None),
                            integ=s["integrity"], alive=s["alive"],
                            loop_ms=round(1000.0 * loop_acc / max(loop_n, 1), 2)))
            loop_acc, loop_n = 0.0, 0
            if len(log) % 250 == 0:
                print(json.dumps({k: log[-1][k] for k in
                                  ("t", "phase", "h", "speed", "fuel", "thr",
                                   "sweep", "loop_ms")}), flush=True)
    finally:
        try:
            client.send_indexes([(roles.slider_index[(g, "fthrust")], 0.0)
                                 for g in roles.engines])
            client.send_indexes([])
        finally:
            landing_err = None
            if final_root_pos is not None:
                a1 = unit(final_root_pos - CENTER)
                a2 = unit(launch_pos - CENTER)
                landing_err = R_SURFACE * math.acos(
                    max(-1.0, min(1.0, float(a1 @ a2))))
            summary = dict(
                schema="s02.mission.v1",
                controller="controller_mission.py",
                phases=events, abort=abort, frames=len(log),
                sweep_deg=round(sweep_deg, 2),
                payload_rmin=(round(payload_rmin, 1) if payload_rmin < 1e11 else None),
                rmin_qualified=rmin_qualified,
                payload_orbit=payload_orbit,
                landing_err=landing_err,
                launch_pos=launch_pos.tolist(),
                final_root_pos=(final_root_pos.tolist()
                                if final_root_pos is not None else None),
                sep_time=sep_time, landed_time=landed_time,
                fuel_left=(log[-1]["fuel"] if log else None),
                roles=dict(engines=roles.engines, wheels=roles.wheels,
                           root=roles.root, tanks=roles.tanks,
                           decoupler=roles.decoupler,
                           payload=roles.payload_blocks),
                plane_normal=p0.tolist(), up0=up0.tolist(), d0h=d0h.tolist(),
                ground_alt=ground_alt,
            )
            (run_dir / "mission.json").write_text(json.dumps(summary), encoding="utf-8")
            (run_dir / "frames.json").write_text(json.dumps(log), encoding="utf-8")
            (run_dir / "mission_summary.json").write_text(json.dumps(
                dict(done=(phase == "done"), abort_reason=abort, phase=phase,
                     sweep_deg=round(sweep_deg, 1),
                     payload_rmin=(round(payload_rmin, 1)
                                   if payload_rmin < 1e11 else None),
                     landing_err=landing_err, events=events), indent=1),
                encoding="utf-8")
            client.close()


if __name__ == "__main__":
    main()
