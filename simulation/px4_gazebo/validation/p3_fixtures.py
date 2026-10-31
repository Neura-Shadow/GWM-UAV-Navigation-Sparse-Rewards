"""Versioned P3 ground fixture worlds generated from the pinned PX4 default world.

The generator only appends static boxes and the x500_depth include; it never
changes vehicle, sensor, physics or rendering settings from the pinned assets.
Geometry is declared by the visible front face (negative local-X face) to keep
the analytic distance definition explicit. The independent evaluator re-reads
the generated SDF it was given; it does not import these declarations.
"""
import math
import xml.etree.ElementTree as ET

FIXTURES = ("plane2", "plane4", "plane6", "oblique", "asymmetric", "out_of_range", "interruption")
GROUND_WORLD = "gwm_p3_ground"
MODEL = "x500_depth_71"


def geometry_case(name):
    if name not in FIXTURES:
        raise ValueError("Unknown P3 fixture: " + str(name))
    return "plane4" if name == "interruption" else name


def box(world, name, front, size, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    center = [front[0] + c * size[0] / 2, front[1] + s * size[0] / 2, front[2]]
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(repr(float(v)) for v in center + [0, 0, yaw])
    link = ET.SubElement(model, "link", name="fixture_link")
    for kind in ("visual", "collision"):
        part = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(part, "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(repr(float(v)) for v in size)
    return {"name": name, "front_m": list(front), "center_m": center, "size_m": list(size), "yaw_rad": yaw}


def generate(base_world_text, fixture, validation, optical_forward_offset_m):
    """Return (sdf_text, declaration). `optical_forward_offset_m` is the pinned optical X."""
    case = geometry_case(fixture)
    spec = validation["fixtures"][case]
    root = ET.fromstring(base_world_text)
    world = root.find("world")
    if world is None or world.find("model[@name='%s']" % MODEL) is not None:
        raise ValueError("Unexpected pinned base world structure")
    world.set("name", GROUND_WORLD)
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://x500_depth"
    ET.SubElement(include, "name").text = MODEL
    ET.SubElement(include, "pose").text = " ".join(repr(float(v)) for v in validation["fixture_vehicle_pose_world"])
    height = validation["front_plane_center_height_m"]
    boxes = [box(world, "front_plane", [spec["distance_m"] + optical_forward_offset_m, 0.0, height],
                 validation["front_plane_size_m"], spec["yaw_rad"])]
    for name, extra in spec.get("boxes", {}).items():
        boxes.append(box(world, name, extra["front_m"], extra["size_m"]))
    text = ET.tostring(root, encoding="unicode")
    return text, {"fixture": fixture, "geometry_case": case, "world": GROUND_WORLD, "model": MODEL,
                  "vehicle_pose_world": validation["fixture_vehicle_pose_world"], "boxes": boxes,
                  "nominal_distance_m": spec["distance_m"], "plane_yaw_rad": spec["yaw_rad"]}


def scene_change_pose(validation, optical_forward_offset_m):
    """Requested front_plane pose for the post-window 4 m -> 5 m render challenge."""
    change = validation["scene_change"]
    size = validation["front_plane_size_m"]
    return {"name": "front_plane", "position_m": [change["to_distance_m"] + optical_forward_offset_m + size[0] / 2,
                                                  0.0, validation["front_plane_center_height_m"]]}
