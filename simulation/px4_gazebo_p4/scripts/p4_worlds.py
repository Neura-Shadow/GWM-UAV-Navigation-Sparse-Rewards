"""P4 scenario worlds generated from the pinned PX4 default world (contract §7).

Only static obstacle boxes and a contact sensor on each obstacle are appended;
the vehicle (spawned by PX4), sensors, physics and rendering settings stay
exactly as pinned. No world plugin is added: the pinned world declares none, so
Gazebo loads every system (Physics, SceneBroadcaster, Contact, Sensors, ...)
from PX4's server.config, and a single world-level plugin would replace that
whole set (P4 Gate A 4722a88e). The world keeps the name `default` so PX4's
bridge and topics are unchanged. The evaluator re-reads the declaration written
next to the world; nothing on the vehicle reads it.
"""
import xml.etree.ElementTree as ET

CONTACT_SYSTEM = "gz-sim-contact-system"  # supplied by PX4's server.config, never by the world
CONTACT_PREFIX = "/gwm/p4/contacts/"
WORLD = "default"


def scenario_spec(scenarios, name):
    for group in ("scenarios", "ground_fixtures"):
        if name in scenarios[group]:
            return group, scenarios[group][name]
    raise ValueError("Unknown P4 scenario: " + str(name))


def _box(world, name, center, size, height, center_z):
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(repr(float(v)) for v in (center[0], center[1], center_z, 0, 0, 0))
    link = ET.SubElement(model, "link", name="obstacle_link")
    dims = " ".join(repr(float(v)) for v in (size[0], size[1], height))
    for kind in ("visual", "collision"):
        part = ET.SubElement(link, kind, name=kind)
        ET.SubElement(ET.SubElement(ET.SubElement(part, "geometry"), "box"), "size").text = dims
    sensor = ET.SubElement(link, "sensor", name="contact", type="contact")
    ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = "collision"
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "100"
    ET.SubElement(sensor, "topic").text = CONTACT_PREFIX + name
    return {"name": name, "center_m": [float(center[0]), float(center[1]), float(center_z)],
            "size_m": [float(size[0]), float(size[1]), float(height)], "contact_topic": CONTACT_PREFIX + name}


def generate(base_world_text, name, scenarios):
    """Return (sdf_text, declaration) for scenario S1-S6 or ground fixture G2/G4/G6."""
    group, spec = scenario_spec(scenarios, name)
    root = ET.fromstring(base_world_text)
    world = root.find("world")
    if world is None or world.get("name") != WORLD:
        raise ValueError("Unexpected pinned base world structure")
    if any(m.get("name", "").startswith(("pillar", "wall")) for m in world.findall("model")):
        raise ValueError("Base world already contains P4 obstacles")
    if world.find("plugin") is not None:
        raise ValueError("Pinned base world must not declare plugins; PX4 server.config supplies the systems")
    boxes = [_box(world, b["name"], b["center"], b["size"], scenarios["box_height_m"], scenarios["box_center_height_m"])
             for b in spec["boxes"]]
    declaration = {"contract": "p4-scenario-declaration-v1", "scenario": name, "group": group, "world": WORLD,
                   "frame": scenarios["frame"], "boxes": boxes,
                   "outcome": spec.get("outcome"), "fault": spec.get("fault"),
                   "expected_trigger": spec.get("expected_trigger"), "detour_side": spec.get("detour_side")}
    return ET.tostring(root, encoding="unicode"), declaration
