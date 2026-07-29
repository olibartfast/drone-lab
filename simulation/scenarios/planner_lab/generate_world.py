#!/usr/bin/env python3
"""Convert the immutable planner report into a deterministic Gazebo SDF scene."""

import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


COLOURS = {
    "source": "0.20 0.20 0.20 1.0",
    "inflated": "0.85 0.25 0.15 0.75",
    "start": "0.10 0.85 0.20 1.0",
    "goal": "0.15 0.30 0.95 1.0",
    "raw": "0.95 0.75 0.10 1.0",
    "pruned": "0.05 0.90 0.90 1.0",
}


def cell_position(report: dict, cell: dict) -> tuple[float, float]:
    metric_map = report["map"]
    resolution = float(metric_map["resolution_m"])
    origin = metric_map["origin_m"]
    return (
        float(origin["x"]) + (int(cell["column"]) + 0.5) * resolution,
        float(origin["y"]) + (int(cell["row"]) + 0.5) * resolution,
    )


def add_box(world: ET.Element, name: str, x: float, y: float, z: float,
            sx: float, sy: float, sz: float, colour: str) -> None:
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:.6f} {y:.6f} {z:.6f} 0 0 0"
    link = ET.SubElement(model, "link", name="link")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = f"{sx:.6f} {sy:.6f} {sz:.6f}"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = colour
    ET.SubElement(material, "diffuse").text = colour


def add_sphere(world: ET.Element, name: str, x: float, y: float,
               z: float, radius: float, colour: str) -> None:
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:.6f} {y:.6f} {z:.6f} 0 0 0"
    link = ET.SubElement(model, "link", name="link")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = f"{radius:.6f}"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = colour
    ET.SubElement(material, "diffuse").text = colour


def add_segment(world: ET.Element, name: str, a: tuple[float, float],
                b: tuple[float, float], z: float, radius: float, colour: str) -> None:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    yaw = math.atan2(dy, dx)
    ET.SubElement(model, "pose").text = (
        f"{(a[0] + b[0]) * 0.5:.6f} {(a[1] + b[1]) * 0.5:.6f} "
        f"{z:.6f} 0 {math.pi * 0.5:.6f} {yaw:.6f}"
    )
    link = ET.SubElement(model, "link", name="link")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = f"{radius:.6f}"
    ET.SubElement(cylinder, "length").text = f"{length:.6f}"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = colour
    ET.SubElement(material, "diffuse").text = colour


def generate(report: dict) -> tuple[str, dict]:
    required = {
        "schema_version", "scenario", "map", "source_blocked_cells",
        "inflated_blocked_cells", "start", "goal", "raw_path", "pruned_path",
        "status", "rejection_reason",
    }
    missing = sorted(required - report.keys())
    if missing or report.get("schema_version") != 1:
        raise ValueError(f"marker_contract_mismatch: missing={missing}")
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="planner_lab")
    ET.SubElement(world, "gravity").text = "0 0 0"
    physics = ET.SubElement(world, "physics", name="fixed", type="dart")
    ET.SubElement(physics, "max_step_size").text = "0.0166666666666667"
    ET.SubElement(physics, "real_time_update_rate").text = "60"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
    ):
        plugin = ET.SubElement(world, "plugin", filename=filename, name=name)
        if "scene-broadcaster" in filename:
            ET.SubElement(plugin, "state_hertz").text = "30"
        if "sensors" in filename:
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.65 0.65 0.65 1"
    ET.SubElement(scene, "background").text = "0.08 0.10 0.14 1"
    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "direction").text = "-0.4 0.2 -1"
    metric_map = report["map"]
    resolution = float(metric_map["resolution_m"])
    origin = metric_map["origin_m"]
    map_width = int(metric_map["width"]) * resolution
    map_height = int(metric_map["height"]) * resolution
    add_box(
        world, "map_floor", float(origin["x"]) + map_width * 0.5,
        float(origin["y"]) + map_height * 0.5, -0.035,
        map_width, map_height, 0.05, "0.75 0.78 0.82 1",
    )
    source_cells = {
        (int(cell["row"]), int(cell["column"])) for cell in report["source_blocked_cells"]
    }
    for index, cell in enumerate(report["inflated_blocked_cells"]):
        key = (int(cell["row"]), int(cell["column"]))
        if key in source_cells:
            continue
        x, y = cell_position(report, cell)
        add_box(world, f"inflated_{index:04d}", x, y, 0.02,
                resolution * 0.96, resolution * 0.96, 0.04, COLOURS["inflated"])
    for index, cell in enumerate(report["source_blocked_cells"]):
        x, y = cell_position(report, cell)
        add_box(world, f"source_{index:04d}", x, y, 0.08,
                resolution * 0.92, resolution * 0.92, 0.16, COLOURS["source"])
    for label in ("start", "goal"):
        x, y = cell_position(report, report[label])
        add_sphere(world, label, x, y, 0.24, resolution * 0.25, COLOURS[label])
    for kind, z, radius in (("raw", 0.18, 0.035), ("pruned", 0.28, 0.055)):
        path = [cell_position(report, cell) for cell in report[f"{kind}_path"]]
        for index, (a, b) in enumerate(zip(path, path[1:])):
            add_segment(world, f"{kind}_{index:04d}", a, b, z, radius, COLOURS[kind])
    status_colour = "0.10 0.75 0.20 1" if report["status"] == "success" else "0.90 0.10 0.10 1"
    add_box(world, "plan_status", float(origin["x"]) + map_width * 0.5,
            float(origin["y"]) + map_height + 0.35, 0.08,
            map_width, 0.25, 0.16, status_colour)

    camera_model = ET.SubElement(world, "model", name="fixed_camera")
    ET.SubElement(camera_model, "static").text = "true"
    ET.SubElement(camera_model, "pose").text = "2.0 0.25 14.0 0 1.570796 1.570796"
    camera_link = ET.SubElement(camera_model, "link", name="link")
    sensor = ET.SubElement(camera_link, "sensor", name="planner_camera", type="camera")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "30"
    ET.SubElement(sensor, "topic").text = "/planner_lab/camera/image"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = "1.0471975512"
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = "1280"
    ET.SubElement(image, "height").text = "720"
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "50"
    recorder = ET.SubElement(
        sensor, "plugin", filename="gz-sim-camera-video-recorder-system",
        name="gz::sim::systems::CameraVideoRecorder",
    )
    ET.SubElement(recorder, "service").text = "/planner_lab/record_video"
    ET.SubElement(recorder, "use_sim_time").text = "true"
    ET.SubElement(recorder, "fps").text = "30"
    ET.SubElement(recorder, "bitrate").text = "4000000"
    marker_counts = {
        "source": len(report["source_blocked_cells"]),
        "inflated": len(report["inflated_blocked_cells"]) - len(source_cells),
        "start_goal": 2,
        "raw_segments": max(0, len(report["raw_path"]) - 1),
        "pruned_segments": max(0, len(report["pruned_path"]) - 1),
        "status": 1,
    }
    marker_counts["total"] = sum(marker_counts.values())
    xml = ET.tostring(sdf, encoding="unicode", xml_declaration=True)
    return xml + "\n", marker_counts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("planner_report", type=Path)
    parser.add_argument("world_output", type=Path)
    parser.add_argument("marker_output", type=Path)
    args = parser.parse_args()
    try:
        report = json.loads(args.planner_report.read_text(encoding="utf-8"))
        world, marker_counts = generate(report)
        args.world_output.write_text(world, encoding="utf-8")
        args.marker_output.write_text(
            json.dumps({"schema_version": 1, "marker_counts": marker_counts}, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(json.dumps({"status": "failed", "failure_reason": "marker_contract_mismatch",
                          "message": str(error)}, sort_keys=True))
        return 1
    print(json.dumps({"status": "passed", "marker_counts": marker_counts}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
