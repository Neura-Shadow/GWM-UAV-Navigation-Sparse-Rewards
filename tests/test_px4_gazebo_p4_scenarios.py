"""P4 M5: scenario worlds (contract §7), ground fixtures and the truth observer, offline."""
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
import p4_truth_observer  # noqa: E402
import p4_worlds  # noqa: E402

BASE = ('<sdf version="1.9"><world name="default"><physics name="1ms" type="ignored"/>'
        '<model name="ground_plane"><static>true</static><link name="link"><collision name="c"><geometry><plane>'
        '<normal>0 0 1</normal></plane></geometry></collision></link></model></world></sdf>')
SCENARIOS = syn.scenarios()

CONTRACT_TABLE = {  # contract §7: centre (x, y) and size (x, y); every box 4 m tall, centred at 2 m
    "S1": [],
    "S2": [((4.0, 0.0), (0.4, 0.4))],
    "S3": [((4.0, 0.5), (0.4, 0.4))],
    "S4": [((3.5, 0.0), (0.4, 0.4)), ((6.0, -1.4), (0.4, 0.4))],
    "S5": [((4.0, 0.0), (0.3, 8.0))],
    "S6": [((4.0, 0.0), (0.4, 0.4))],
}


@pytest.mark.parametrize("name", sorted(CONTRACT_TABLE))
def test_scenario_world_matches_the_contract_table(name):
    text, declaration = p4_worlds.generate(BASE, name, SCENARIOS)
    world = ET.fromstring(text).find("world")
    assert world.get("name") == "default"
    assert world.find("include") is None  # PX4 spawns the vehicle; the vehicle SDF is untouched
    assert world.find("physics") is not None and world.find("model[@name='ground_plane']") is not None
    got = [(tuple(b["center_m"][:2]), tuple(b["size_m"][:2])) for b in declaration["boxes"]]
    assert got == CONTRACT_TABLE[name]
    for box in declaration["boxes"]:
        assert box["size_m"][2] == 4.0 and box["center_m"][2] == 2.0
        model = world.find(f"model[@name='{box['name']}']")
        assert model.findtext("static") == "true"
        pose = [float(v) for v in model.findtext("pose").split()]
        assert pose[:3] == box["center_m"] and pose[3:] == [0, 0, 0]
        size = [float(v) for v in model.find("link/collision/geometry/box/size").text.split()]
        assert size == box["size_m"]
        assert model.find("link/sensor[@type='contact']/topic").text == "/gwm/p4/contacts/" + box["name"]
    plugins = [p.get("filename") for p in world.findall("plugin")]
    assert ("gz-sim-contact-system" in plugins) == True  # noqa: E712
    assert declaration["outcome"] == SCENARIOS["scenarios"][name]["outcome"]


def test_s6_declares_the_sensor_fault_and_s3_the_expected_side():
    assert p4_worlds.generate(BASE, "S6", SCENARIOS)[1]["fault"] == "bridge_stop_at_distance"
    assert p4_worlds.generate(BASE, "S3", SCENARIOS)[1]["detour_side"] == "negative_y"
    assert all(p4_worlds.generate(BASE, n, SCENARIOS)[1]["fault"] is None for n in ("S1", "S2", "S3", "S4", "S5"))


@pytest.mark.parametrize("name,trigger,nearest", [("G2", "obstacle_in_trigger_range", 1.8),
                                                  ("G4", "no_obstacle_in_trigger_range", 3.8),
                                                  ("G6", "no_obstacle_in_trigger_range", 5.85)])
def test_ground_fixtures_place_obstacles_at_2_4_and_6_m(name, trigger, nearest):
    _, declaration = p4_worlds.generate(BASE, name, SCENARIOS)
    assert declaration["group"] == "ground_fixtures" and declaration["expected_trigger"] == trigger
    surfaces = [b["center_m"][0] - b["size_m"][0] / 2 for b in declaration["boxes"]]
    assert min(surfaces) == pytest.approx(nearest)
    assert (min(surfaces) <= syn.p4_config()["planner"]["trigger_range_m"]) == (trigger == "obstacle_in_trigger_range")


def test_generator_rejects_unknown_scenarios_and_already_modified_worlds():
    with pytest.raises(ValueError, match="Unknown P4 scenario"):
        p4_worlds.generate(BASE, "S9", SCENARIOS)
    text, _ = p4_worlds.generate(BASE, "S2", SCENARIOS)
    with pytest.raises(ValueError, match="already contains"):
        p4_worlds.generate(text, "S2", SCENARIOS)
    with pytest.raises(ValueError, match="pinned base world"):
        p4_worlds.generate(BASE.replace('name="default"', 'name="other"'), "S1", SCENARIOS)


def test_truth_rows_keep_only_the_vehicle_and_record_contact_pairs():
    pose = lambda name, x: SimpleNamespace(name=name, position=SimpleNamespace(x=x, y=0.5, z=2.0),
                                           orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))
    row = p4_truth_observer.pose_row(5_000_000_000, [pose("x500_depth_71", 1.0), pose("pillar_a", 4.0)], "x500_depth_71")
    assert list(row["poses"]) == ["x500_depth_71"] and row["poses"]["x500_depth_71"]["position"] == [1.0, 0.5, 2.0]
    assert p4_truth_observer.pose_row(1, [pose("pillar_a", 4.0)], "x500_depth_71") is None
    contact = SimpleNamespace(collision1=SimpleNamespace(name="pillar_a::obstacle_link::collision"),
                              collision2=SimpleNamespace(name="x500_depth_71::base_link::collision"), position=[1, 2])
    row = p4_truth_observer.contact_row(7, "pillar_a", [contact])
    assert row["contacts"] == [["pillar_a::obstacle_link::collision", "x500_depth_71::base_link::collision", 2]]
    assert p4_truth_observer.MIN_POSE_INTERVAL_NS / 1e9 <= 0.05


def test_truth_observer_refuses_outside_the_owned_namespace(tmp_path, monkeypatch):
    monkeypatch.delenv("GWM_P4_OWNED_NAMESPACE", raising=False)
    with pytest.raises(ValueError, match="owned P4 namespace"):
        p4_truth_observer.main([str(tmp_path), "default", "x500_depth_71"])
