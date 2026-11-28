"""P8 recovery manager: the per-HOVER-episode recovery layer (contract `p8-navigation-recovery-v1` §3, §4).

Pure and deterministic. It reads the frozen supervisor's output for a tick and either returns it unchanged or replaces
its HOLD with one recovery action. The frozen supervisor stays authoritative (S4): its state, timers and aborts are
never touched.

**When it acts.** The trigger holds when all of these do:
* the supervisor is in HOVER, entered with the reason `no_progress_candidate`;
* there is no stale reason (S3);
* the frozen action of the tick is HOLD;
* the tick is neither frozen-yaw nor a P5 policy-failure tick;
* the episode's window and budgets are not exhausted (S8).

**What it does:** one pass per episode (`p8_recovery.json` `attempts`).
* **R1:** used near the goal when only UNKNOWN cells block the terminal segment. It yaws toward the blocking cells'
  sectors.
* **R2:** used otherwise. It yaws toward the reachable frontier sectors outside the planner's ±60° cone.
* **R3:** follows the episode search's path, in segments.

**Every action is checked by the frozen functions on the snapshot of that tick, before it is emitted:**
* a yaw by `planner.yaw_permitted` (L5);
* a translation by the frozen L4 rule (heading within `observable_half_angle_deg` of the vehicle yaw; otherwise it
  yaws first) and by `planner.collision_check` (L2, with `r_infl_m` and the fence).

A refused check means HOLD and a ledger record (FC-4). Every action, refusal and episode boundary is appended to
`records` (S10), and the node persists them as `p8/recovery-ledger.jsonl`.
"""
import base64
import math
import time
import zlib

import numpy as np

from gwm_obstacle_avoidance import planner
from gwm_obstacle_avoidance.local_map import OCCUPIED
from gwm_obstacle_avoidance.supervisor import HOVER

from . import search

LEDGER = "p8-recovery-ledger-v1"
TRIGGER_REASON = "no_progress_candidate"
YAW_TOLERANCE_DEG = 5.0  # half the 10 degree sector step: the sector is faced


def encode_mask(mask):
    packed = np.packbits(np.asarray(mask, dtype=bool), axis=None)
    return {"shape": list(mask.shape), "encoding": "packbits-zlib-base64",
            "data": base64.b64encode(zlib.compress(packed.tobytes(), 6)).decode("ascii")}


def decode_mask(value):
    packed = np.frombuffer(zlib.decompress(base64.b64decode(value["data"])), dtype=np.uint8)
    n = value["shape"][0] * value["shape"][1]
    return np.unpackbits(packed)[:n].astype(bool).reshape(value["shape"])


def terminal_blockers(grid, traversed, target, goal, r_infl):
    """What the L2 rule finds within `r_infl` of the terminal segment target -> goal: the OCCUPIED count and the
    centres of the cells that are neither observed FREE nor traversed."""
    ii, jj, states, inside = grid.segment_cells(target, goal, r_infl)
    occupied = states == OCCUPIED
    free = search.known_free(grid, traversed)[ii, jj]
    unknown = ~free & ~occupied
    return int(occupied.sum()), [grid.center(int(i), int(j)) for i, j in zip(ii[unknown], jj[unknown])], bool(inside)


def chain_sectors(headings, start_yaw, budget_rad):
    """Visit order for sector headings: greedy nearest-next from `start_yaw`, truncated to the rotation budget."""
    left, here, used, out = list(headings), start_yaw, 0.0, []
    while left:
        nxt = min(left, key=lambda h: (abs(search.wrap(h - here)), h))
        cost = abs(search.wrap(nxt - here))
        if used + cost > budget_rad + 1e-9:
            break
        used, here = used + cost, nxt
        out.append(nxt)
        left.remove(nxt)
    return out, used


class RecoveryManager:
    def __init__(self, p4_config, params, clock=time.perf_counter):
        self.p4, self.params = p4_config, params
        self.settings, self.envelope = p4_config["planner"], p4_config["envelope"]
        self.max_age = p4_config["perception"]["snapshot_max_age_sim_s"]
        self.r_infl = self.settings["r_infl_m"]
        self.half = math.radians(self.settings["observable_half_angle_deg"])
        self.cone = math.radians(self.settings["heading_step_deg"] * self.settings["heading_steps"])
        self.step_rad = math.radians(params["r2"]["sector_step_deg"])
        ep = params["episode"]
        self.window, self.rate = ep["window_s"], math.radians(ep["yaw_rate_deg_s"])
        self.max_rot, self.max_trans, self.speed = math.radians(ep["max_rotation_deg"]), ep["max_translation_m"], ep["speed_m_s"]
        self.goal_radius = params["r1"]["goal_radius_m"]
        self.segment_max = params["r3"]["segment_max_m"]
        self.dead_radius = params["dead_end_memory"]["radius_m"]
        self.dead_ends = []
        self.episode = None
        self.records = []
        self.clock = clock

    # ---- helpers ---------------------------------------------------------------------------------------------------
    def _ctx(self, inputs):
        age = inputs["sim"] - inputs["snapshot"]["acquisition_sim_s"]
        return planner.Context(inputs["target"], inputs["vehicle"], inputs["vehicle_yaw"], inputs["goal"], inputs["origin"],
                               age, inputs["geofence"])

    def _dead_mask(self, grid):
        mask = np.zeros(grid.cells.shape, dtype=bool)
        if self.dead_ends:
            mask = planner.traversed_mask(grid, self.dead_ends, self.dead_radius)
        return mask

    def _record(self, inputs, kind, **fields):
        snap = inputs.get("snapshot") or {}
        rec = {"ledger": LEDGER, "kind": kind, "sim_s": inputs["sim"],
               "episode_hover_since_sim_s": (self.episode or {}).get("since"),
               "snapshot": {"observation_id": snap.get("observation_id"), "acquisition_sim_s": snap.get("acquisition_sim_s")},
               "target": list(inputs["target"]), "vehicle": list(inputs["vehicle"]), "vehicle_yaw": inputs["vehicle_yaw"],
               "goal": list(inputs["goal"]), "origin": list(inputs["origin"])}
        rec.update(fields)
        self.records.append(rec)
        return rec

    def _close(self, inputs, why):
        if self.episode is not None:
            e = self.episode
            self._record(inputs, "episode_end", why=why, rotation_used_rad=e["rot"], translation_used_m=e["trans"],
                         actions=e["actions"], refusals=e["refusals"])
            self.episode = None

    def _open(self, inputs, since):
        grid, traversed = inputs["grid"], inputs["traversed"]
        target, goal = inputs["target"], inputs["goal"]
        self.dead_ends.append(tuple(target))
        found = search.episode_search(grid, traversed, target, goal, inputs["origin"], inputs["geofence"], self.r_infl,
                                      self._dead_mask(grid), self.clock)
        elapsed = inputs["sim"] - since
        left = max(0.0, self.window - elapsed)  # the time rule: rotation time + translation time <= the window left
        yaw0 = inputs["vehicle_yaw"]
        queue, phases = [], {}
        occupied, unknown, _ = terminal_blockers(grid, traversed, target, goal, self.r_infl)
        r1 = math.dist(target, goal) <= self.goal_radius + 1e-9 and occupied == 0 and len(unknown) > 0
        ok = search.c_space(grid, traversed, self.r_infl, inputs["origin"], inputs["geofence"])

        def segment_ok(p, q):
            steps = max(2, int(math.dist(p, q) / (grid.resolution / 2)) + 1)
            for n in range(steps + 1):
                t = n / steps
                cell = grid.index(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))
                if cell is None or not ok[cell]:
                    return False
            return True
        ends = search.waypoints(grid, found["path"], tuple(target), self.segment_max, segment_ok)

        def r3_cost(prefix, start_yaw):
            """(time, rotation, length) of R3 segments from `start_yaw`, with the L4 alignment yaws."""
            t = rot = length = 0.0
            here, y = tuple(target), start_yaw
            for end in prefix:
                h = search.bearing(here, end)
                turn = max(0.0, abs(search.wrap(h - y)) - self.half)
                d = math.dist(here, end)
                t, rot, length = t + turn / self.rate + d / self.speed, rot + turn, length + d
                here, y = end, h
            return t, rot, length

        def r3_prefix(start_yaw, time_left, rot_left):
            kept = []
            for end in ends:
                t, rot, length = r3_cost(kept + [end], start_yaw)
                if t > time_left + 1e-9 or rot > rot_left + 1e-9 or length > self.max_trans + 1e-9:
                    break
                kept.append(end)
            return kept
        if r1:
            # R1 (an observation problem): its scans first, then R3 in the time and rotation left (interpretation I-1).
            heads = sorted({search.quantize(search.bearing(target, c), self.step_rad) for c in unknown})
            order, used = chain_sectors(heads, yaw0, min(self.max_rot, left * self.rate))
            last = order[-1] if order else yaw0
            kept = r3_prefix(last, left - used / self.rate, self.max_rot - used)
            queue += [("yaw", h, "R1") for h in order]
            phases["R1"] = {"sectors": len(heads), "planned": len(order), "rotation_rad": used}
        else:
            # R2 (a motion problem): the escape's time is reserved first, then R2 sectors fill the rest (interpretation I-1).
            kept = r3_prefix(yaw0, left, self.max_rot)
            path_heading = search.bearing(target, goal)
            sectors = {}
            ii, jj = np.nonzero(found["frontier"])
            for i, j in zip(ii.tolist(), jj.tolist()):
                c = grid.center(i, j)
                if math.dist(c, target) < grid.resolution:
                    continue
                b = search.bearing(target, c)
                if abs(search.wrap(b - path_heading)) <= self.cone + 1e-9:
                    continue
                h = search.quantize(b, self.step_rad)
                d = float(found["goal_distance"][i, j])
                sectors[h] = min(d, sectors.get(h, math.inf))
            ranked = [h for h, _ in sorted(sectors.items(), key=lambda kv: (kv[1], kv[0]))]
            order, used, here = [], 0.0, yaw0
            for h in ranked:
                cost = abs(search.wrap(h - here))
                t3, rot3, _ = r3_cost(kept, h)
                if (used + cost) / self.rate + t3 > left + 1e-9 or used + cost + rot3 > self.max_rot + 1e-9:
                    continue
                used, here = used + cost, h
                order.append(h)
            queue += [("yaw", h, "R2") for h in order]
            phases["R2"] = {"sectors": len(sectors), "planned": len(order), "rotation_rad": used}
            last = order[-1] if order else yaw0
        t3, rot3, length = r3_cost(kept, last)
        queue += [("translate", tuple(end), "R3") for end in kept]
        phases["R3"] = {"target_kind": found["target_kind"], "path_cells": len(found["path"] or []), "segments_found": len(ends),
                        "segments": len(kept), "length_m": length, "alignment_rotation_rad": rot3, "time_s": t3}
        plan_time = phases.get("R1", phases.get("R2"))["rotation_rad"] / self.rate + t3
        self.episode = {"since": since, "queue": queue, "rot": 0.0, "trans": 0.0, "last_yaw": inputs["vehicle_yaw"],
                        "last_target": tuple(target), "actions": 0, "refusals": 0}
        self._record(inputs, "episode_start", r1_applies=r1, terminal_occupied=occupied, terminal_unknown=len(unknown),
                     search={"wall_s": found["wall_s"], "start_usable": found["start_usable"],
                             "reachable_cells": int(found["reach"].sum()), "frontier_cells": int(found["frontier"].sum()),
                             "target_kind": found["target_kind"]},
                     phases=phases, planned_time_s=plan_time, window_left_s=left,
                     plan_fits_window=plan_time <= left + 1e-9,
                     queue=[[k, list(v) if isinstance(v, tuple) else v, ph] for k, v, ph in queue])

    def _account(self, inputs):
        e = self.episode
        e["rot"] += abs(search.wrap(inputs["vehicle_yaw"] - e["last_yaw"]))
        e["trans"] += math.dist(inputs["target"], e["last_target"])
        e["last_yaw"], e["last_target"] = inputs["vehicle_yaw"], tuple(inputs["target"])

    # ---- the tick --------------------------------------------------------------------------------------------------
    def step(self, inputs, out, hover):
        """`hover` is (hover_since_sim_s, entry_reason) of the frozen supervisor's current HOVER, or None."""
        policy = out.get("policy") or {}
        triggered = (out["state"] == HOVER and out["stale"] is None and hover is not None and hover[1] == TRIGGER_REASON
                     and not out["frozen_yaw"] and policy.get("failure") is None and inputs.get("grid") is not None
                     and inputs.get("snapshot") is not None)
        if not triggered:
            if self.episode is not None and (out["state"] != HOVER or hover is None or hover[0] != self.episode["since"]):
                self._close(inputs, "left_hover" if out["state"] != "ABORT" else "abort")
            elif self.episode is not None and out["stale"] is not None:
                self._record(inputs, "inactive", why="stale:" + str(out["stale"]))
            return out
        if self.episode is not None and self.episode["since"] != hover[0]:
            self._close(inputs, "new_episode")
        if self.episode is None:
            self._open(inputs, hover[0])
        else:
            self._account(inputs)
        e = self.episode
        if out["action"] != planner.HOLD:
            return out  # the frozen SCAN of the tick goes first (S4)
        if inputs["sim"] - e["since"] >= self.window:
            return out
        grid, traversed, ctx = inputs["grid"], inputs["traversed"], self._ctx(inputs)
        while e["queue"]:
            kind, value, phase = e["queue"][0]
            if kind == "yaw":
                if abs(search.wrap(inputs["vehicle_yaw"] - value)) <= math.radians(YAW_TOLERANCE_DEG):
                    e["queue"].pop(0)
                    continue
                return self._yaw(inputs, out, grid, traversed, ctx, value, phase)
            end = value
            if math.dist(inputs["target"], end) < 1e-6:
                e["queue"].pop(0)
                continue
            if e["trans"] >= self.max_trans - 1e-9:
                e["queue"] = [q for q in e["queue"] if q[0] != "translate"]
                continue
            heading = search.bearing(inputs["target"], end)
            if abs(search.wrap(heading - inputs["vehicle_yaw"])) > self.half:
                return self._yaw(inputs, out, grid, traversed, ctx, heading, phase, for_translation=True)
            valid, reason, unknown_only, unknown = planner.collision_check(grid, traversed, inputs["target"], end, ctx,
                                                                           self.settings, self.max_age)
            if not valid:
                e["refusals"] += 1
                e["queue"] = [q for q in e["queue"] if q[0] != "translate"]
                self._record(inputs, "refused", phase=phase, action=planner.TRANSLATE, heading=heading, end=list(end),
                             check={"function": "planner.collision_check", "result": [valid, reason, unknown_only, unknown]},
                             traversed=encode_mask(traversed), snapshot_age_s=ctx.snapshot_age)
                return out
            e["actions"] += 1
            length = math.dist(inputs["target"], end)
            self._record(inputs, "action", phase=phase, action=planner.TRANSLATE, heading=heading, end=list(end),
                         length=length, check={"function": "planner.collision_check", "result": [valid, reason, unknown_only, unknown]},
                         traversed=encode_mask(traversed), snapshot_age_s=ctx.snapshot_age,
                         rotation_used_rad=e["rot"], translation_used_m=e["trans"])
            return dict(out, action=planner.TRANSLATE, heading=heading, end=end, length=length, reason="p8_recovery:" + phase)
        return out

    def _yaw(self, inputs, out, grid, traversed, ctx, heading, phase, for_translation=False):
        e = self.episode
        if e["rot"] >= self.max_rot - 1e-9:
            e["queue"] = [q for q in e["queue"] if q[0] == "translate" and not for_translation] if not for_translation else []
            return out
        permitted, reason = planner.yaw_permitted(grid, traversed, ctx, self.settings, self.envelope, self.max_age)
        record = dict(phase=phase, action=planner.YAW_TO, heading=heading, for_translation=for_translation,
                      check={"function": "planner.yaw_permitted", "result": [permitted, reason]},
                      traversed=encode_mask(traversed), snapshot_age_s=ctx.snapshot_age,
                      rotation_used_rad=e["rot"], translation_used_m=e["trans"])
        if not permitted:
            e["refusals"] += 1
            e["queue"].pop(0)
            if for_translation:
                e["queue"] = [q for q in e["queue"] if q[0] != "translate"]
            self._record(inputs, "refused", **record)
            return out
        e["actions"] += 1
        self._record(inputs, "action", **record)
        return dict(out, action=planner.YAW_TO, heading=heading, end=None, length=None, reason="p8_recovery:" + phase)
