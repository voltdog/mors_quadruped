#!/usr/bin/env python3
"""Generate an MJCF scene with stairs, stump field and descending stairs."""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_OUTPUT = Path(__file__).with_name("stumps.xml")
FIRST_STEP_START_X = 0.5
MIN_STUMP_TOP_HEIGHT = 0.005
EPS = 1e-9
HEARTBEAT_INTERVAL_SEC = 1.5


@dataclass(frozen=True)
class Stump:
    x: float
    y: float
    radius: float
    top_height: float


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be greater than or equal to zero")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text else "0"


def maybe_heartbeat(last_ts: float, message: str) -> float:
    now = time.monotonic()
    if now - last_ts >= HEARTBEAT_INTERVAL_SEC:
        print(f"[generate_stumps] {message}", file=sys.stderr, flush=True)
        return now
    return last_ts


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an MJCF scene: stairs up, top buffer, random stumps, "
            "top buffer, stairs down."
        )
    )
    parser.add_argument(
        "--platform-width",
        required=True,
        type=positive_float,
        help="Total width of all elevated sections along Y in meters.",
    )
    parser.add_argument(
        "--step-count",
        required=True,
        type=positive_int,
        help="Number of ascending and descending steps.",
    )
    parser.add_argument(
        "--step-height",
        required=True,
        type=positive_float,
        help="Height increment per step in meters.",
    )
    parser.add_argument(
        "--step-depth",
        required=True,
        type=positive_float,
        help="Length of each step segment along X in meters.",
    )
    parser.add_argument(
        "--top-platform-length",
        required=True,
        type=positive_float,
        help=(
            "Length of the flat top buffer before the stump zone in meters. "
            "The same length is used after the stump zone."
        ),
    )
    parser.add_argument(
        "--stump-zone-length",
        required=True,
        type=positive_float,
        help="Length of the stump zone along X in meters.",
    )
    parser.add_argument(
        "--max-stump-gap",
        required=True,
        type=positive_float,
        help=(
            "Maximum allowed edge-to-edge gap from any sampled point in the stump zone "
            "to the nearest stump surface, in meters."
        ),
    )
    parser.add_argument(
        "--min-stump-gap",
        default=0.0,
        type=nonnegative_float,
        help="Minimum allowed edge-to-edge gap between stump surfaces in meters.",
    )
    parser.add_argument(
        "--stump-height-variation",
        required=True,
        type=nonnegative_float,
        help=(
            "Top-height variation of stumps around the top-step height in meters. "
            "Each stump top is sampled from [H_top-variation, H_top+variation]."
        ),
    )
    parser.add_argument(
        "--min-stump-diameter",
        required=True,
        type=positive_float,
        help="Minimum stump diameter in meters.",
    )
    parser.add_argument(
        "--max-stump-diameter",
        required=True,
        type=positive_float,
        help="Maximum stump diameter in meters.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="Optional random seed for reproducible generation.",
    )
    return parser


def make_ascending_step_geoms(
    step_count: int,
    step_height: float,
    step_depth: float,
    platform_half_width: float,
) -> tuple[str, float]:
    geom_lines: list[str] = []
    for index in range(step_count):
        top_height = step_height * (index + 1)
        center_x = FIRST_STEP_START_X + step_depth * (index + 0.5)
        half_top_height = top_height / 2.0
        geom_lines.extend(
            [
                "    <geom",
                f'      name="up_step_{index + 1}"',
                '      type="box"',
                f'      pos="{fmt(center_x)} 0 {fmt(half_top_height)}"',
                f'      size="{fmt(step_depth / 2.0)} {fmt(platform_half_width)} {fmt(half_top_height)}"',
                '      contype="1"',
                '      conaffinity="1"',
                '      material="stairs_geom"',
                "    />",
            ]
        )
    section_end_x = FIRST_STEP_START_X + step_count * step_depth
    return "\n".join(geom_lines), section_end_x


def make_descending_step_geoms(
    start_x: float,
    step_count: int,
    step_height: float,
    step_depth: float,
    platform_half_width: float,
) -> tuple[str, float]:
    geom_lines: list[str] = []
    for index in range(step_count):
        top_height = step_height * (step_count - index)
        center_x = start_x + step_depth * (index + 0.5)
        half_top_height = top_height / 2.0
        geom_lines.extend(
            [
                "    <geom",
                f'      name="down_step_{index + 1}"',
                '      type="box"',
                f'      pos="{fmt(center_x)} 0 {fmt(half_top_height)}"',
                f'      size="{fmt(step_depth / 2.0)} {fmt(platform_half_width)} {fmt(half_top_height)}"',
                '      contype="1"',
                '      conaffinity="1"',
                '      material="stairs_geom"',
                "    />",
            ]
        )
    section_end_x = start_x + step_count * step_depth
    return "\n".join(geom_lines), section_end_x


def make_top_buffer_geom(
    name: str,
    start_x: float,
    length: float,
    top_height: float,
    platform_half_width: float,
) -> tuple[str, float]:
    center_x = start_x + length / 2.0
    half_top_height = top_height / 2.0
    geom = "\n".join(
        [
            "    <geom",
            f'      name="{name}"',
            '      type="box"',
            f'      pos="{fmt(center_x)} 0 {fmt(half_top_height)}"',
            f'      size="{fmt(length / 2.0)} {fmt(platform_half_width)} {fmt(half_top_height)}"',
            '      contype="1"',
            '      conaffinity="1"',
            '      material="stairs_geom"',
            "    />",
        ]
    )
    return geom, start_x + length


def sample_stump_top_height(
    rng: random.Random,
    top_step_height: float,
    variation: float,
) -> float:
    sampled = top_step_height + rng.uniform(-variation, variation)
    return max(MIN_STUMP_TOP_HEIGHT, sampled)


def stump_surface_gap(point_x: float, point_y: float, stumps: list[Stump]) -> float:
    if not stumps:
        return math.inf
    return min(
        math.hypot(point_x - stump.x, point_y - stump.y) - stump.radius
        for stump in stumps
    )


def grid_axis(min_value: float, max_value: float, step: float) -> list[float]:
    axis = [min_value]
    cur = min_value
    while cur + step < max_value - EPS:
        cur += step
        axis.append(cur)
    if axis[-1] < max_value - EPS:
        axis.append(max_value)
    else:
        axis[-1] = max_value
    return axis


def control_points(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    spacing: float,
) -> list[tuple[float, float]]:
    xs = grid_axis(x_min, x_max, spacing)
    ys = grid_axis(y_min, y_max, spacing)
    return [(x, y) for x in xs for y in ys]


def build_axis_centers(
    axis_min: float,
    axis_max: float,
    min_radius: float,
    max_coverage_pitch: float,
    min_center_spacing: float,
) -> list[float]:
    span = axis_max - axis_min
    usable = span - 2.0 * min_radius
    if usable < -EPS:
        raise RuntimeError("stump zone is too small for the minimum stump diameter.")
    if usable <= EPS:
        return [(axis_min + axis_max) / 2.0]

    min_intervals = max(1, int(math.ceil(usable / max_coverage_pitch)))
    max_intervals = int(math.floor(usable / min_center_spacing))
    if max_intervals < 1:
        raise RuntimeError(
            "stump-gap constraints are infeasible for this zone size. "
            "Increase stump-zone dimensions, reduce --min-stump-gap or reduce --min-stump-diameter."
        )
    if min_intervals > max_intervals:
        raise RuntimeError(
            "cannot satisfy both coverage and min-gap constraints with grid placement. "
            "Increase --max-stump-gap, increase stump-zone dimensions, or reduce --min-stump-gap."
        )

    intervals = min_intervals
    spacing = usable / intervals
    return [axis_min + min_radius + idx * spacing for idx in range(intervals + 1)]


def generate_stumps(
    rng: random.Random,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    top_step_height: float,
    max_stump_gap: float,
    min_stump_gap: float,
    stump_height_variation: float,
    min_stump_diameter: float,
    max_stump_diameter: float,
) -> list[Stump]:
    heartbeat_ts = time.monotonic()
    print(
        "[generate_stumps] starting stump generation...",
        file=sys.stderr,
        flush=True,
    )

    min_radius = min_stump_diameter / 2.0
    zone_length = x_max - x_min
    zone_width = y_max - y_min
    if min_stump_diameter > zone_length or min_stump_diameter > zone_width:
        raise RuntimeError(
            "min stump diameter is larger than stump-zone dimensions; "
            "cannot place any stump inside the zone."
        )

    max_coverage_pitch = math.sqrt(2.0) * (max_stump_gap + min_radius)
    min_center_spacing = min_stump_diameter + min_stump_gap
    if min_center_spacing - max_coverage_pitch > 1e-9:
        raise RuntimeError(
            "cannot satisfy both min/max stump gap constraints. "
            "Decrease --min-stump-gap, decrease --min-stump-diameter, or increase --max-stump-gap."
        )

    x_centers = build_axis_centers(
        axis_min=x_min,
        axis_max=x_max,
        min_radius=min_radius,
        max_coverage_pitch=max_coverage_pitch,
        min_center_spacing=min_center_spacing,
    )
    y_centers = build_axis_centers(
        axis_min=y_min,
        axis_max=y_max,
        min_radius=min_radius,
        max_coverage_pitch=max_coverage_pitch,
        min_center_spacing=min_center_spacing,
    )

    spacing_x = math.inf if len(x_centers) <= 1 else (x_centers[1] - x_centers[0])
    spacing_y = math.inf if len(y_centers) <= 1 else (y_centers[1] - y_centers[0])
    nearest_center_spacing = min(spacing_x, spacing_y)
    if math.isfinite(nearest_center_spacing):
        radius_global_cap = min(
            max_stump_diameter / 2.0,
            min_radius + max(0.0, (nearest_center_spacing - min_center_spacing) / 2.0),
        )
    else:
        radius_global_cap = max_stump_diameter / 2.0

    centers = [(x, y) for x in x_centers for y in y_centers]
    rng.shuffle(centers)

    stumps: list[Stump] = []
    for idx, (center_x, center_y) in enumerate(centers, start=1):
        heartbeat_ts = maybe_heartbeat(
            heartbeat_ts,
            (
                "grid pass: "
                f"placed={idx - 1}/{len(centers)}, "
                f"grid={len(x_centers)}x{len(y_centers)}"
            ),
        )

        boundary_radius_cap = min(
            center_x - x_min,
            x_max - center_x,
            center_y - y_min,
            y_max - center_y,
            max_stump_diameter / 2.0,
        )
        radius_cap = min(radius_global_cap, boundary_radius_cap)
        if radius_cap + 1e-9 < min_radius:
            raise RuntimeError(
                "no feasible stump radius for grid placement under current constraints. "
                "Try reducing --min-stump-gap or --min-stump-diameter."
            )

        if radius_cap <= min_radius + 1e-9:
            radius = min_radius
        else:
            radius = rng.uniform(min_radius, radius_cap)

        stumps.append(
            Stump(
                x=center_x,
                y=center_y,
                radius=radius,
                top_height=sample_stump_top_height(rng, top_step_height, stump_height_variation),
            )
        )

    verification_points = control_points(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        spacing=max_stump_gap / 4.0,
    )
    worst_gap = -math.inf
    for point_idx, (point_x, point_y) in enumerate(verification_points, start=1):
        heartbeat_ts = maybe_heartbeat(
            heartbeat_ts,
            (
                "verify pass: "
                f"point={point_idx}/{len(verification_points)}, "
                f"stumps={len(stumps)}"
            ),
        )
        worst_gap = max(worst_gap, stump_surface_gap(point_x, point_y, stumps))
    if worst_gap > max_stump_gap + 1e-9:
        raise RuntimeError(
            f"max stump gap constraint not reached (worst gap {worst_gap:.6f} m > {max_stump_gap:.6f} m). "
            "Try reducing min stump diameter, increasing stump-zone size, or increasing max stump gap."
        )

    print(
        "[generate_stumps] generation complete: "
        f"stumps={len(stumps)}, worst_gap={worst_gap:.4f}",
        file=sys.stderr,
        flush=True,
    )
    return stumps


def make_stump_geoms(stumps: list[Stump]) -> str:
    geom_lines: list[str] = []
    for index, stump in enumerate(stumps, start=1):
        half_height = stump.top_height / 2.0
        geom_lines.extend(
            [
                "    <geom",
                f'      name="stump_{index}"',
                '      type="cylinder"',
                f'      pos="{fmt(stump.x)} {fmt(stump.y)} {fmt(half_height)}"',
                f'      size="{fmt(stump.radius)} {fmt(half_height)}"',
                '      contype="1"',
                '      conaffinity="1"',
                '      material="stump_geom"',
                "    />",
            ]
        )
    return "\n".join(geom_lines)


def render_scene(
    platform_width: float,
    step_count: int,
    step_height: float,
    step_depth: float,
    top_platform_length: float,
    stump_zone_length: float,
    max_stump_gap: float,
    min_stump_gap: float,
    stump_height_variation: float,
    min_stump_diameter: float,
    max_stump_diameter: float,
    seed: int | None,
) -> tuple[str, int]:
    rng = random.Random(seed)

    platform_half_width = platform_width / 2.0
    top_step_height = step_count * step_height

    up_steps, up_end_x = make_ascending_step_geoms(
        step_count=step_count,
        step_height=step_height,
        step_depth=step_depth,
        platform_half_width=platform_half_width,
    )
    pre_buffer, pre_buffer_end_x = make_top_buffer_geom(
        name="top_buffer_before_stumps",
        start_x=up_end_x,
        length=top_platform_length,
        top_height=top_step_height,
        platform_half_width=platform_half_width,
    )

    stump_zone_start_x = pre_buffer_end_x
    stump_zone_end_x = stump_zone_start_x + stump_zone_length
    stumps = generate_stumps(
        rng=rng,
        x_min=stump_zone_start_x,
        x_max=stump_zone_end_x,
        y_min=-platform_half_width,
        y_max=platform_half_width,
        top_step_height=top_step_height,
        max_stump_gap=max_stump_gap,
        min_stump_gap=min_stump_gap,
        stump_height_variation=stump_height_variation,
        min_stump_diameter=min_stump_diameter,
        max_stump_diameter=max_stump_diameter,
    )
    stump_geoms = make_stump_geoms(stumps)

    post_buffer, post_buffer_end_x = make_top_buffer_geom(
        name="top_buffer_after_stumps",
        start_x=stump_zone_end_x,
        length=top_platform_length,
        top_height=top_step_height,
        platform_half_width=platform_half_width,
    )
    down_steps, scene_end_x = make_descending_step_geoms(
        start_x=post_buffer_end_x,
        step_count=step_count,
        step_height=step_height,
        step_depth=step_depth,
        platform_half_width=platform_half_width,
    )

    max_stump_top = max((stump.top_height for stump in stumps), default=top_step_height)
    max_scene_height = max(top_step_height, max_stump_top)
    scene_center_x = (FIRST_STEP_START_X + scene_end_x) / 2.0
    ground_half_x = max(15.0, scene_end_x + 2.0)
    ground_half_y = max(15.0, platform_half_width + 2.0)
    statistic_center_z = max(0.1, max_scene_height / 2.0)
    # statistic_extent = max(ground_half_x, ground_half_y, max_scene_height + 2.0)
    statistic_extent = 2

    return (
        f"""<mujoco model="{{name}} stumps scene">
  <include file="{{path}}" />

  <statistic center="{fmt(scene_center_x)} 0 {fmt(statistic_center_z)}" extent="{fmt(statistic_extent)}" meansize="0.04" />

  <visual>
    <headlight diffuse="0.0 0.0 0.0" ambient="0.0 0.0 0.0" specular="0.0 0.0 0.0" />
    <global azimuth="220" elevation="-10" />
    <quality shadowsize="8192" />
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="edge" markrgb=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="15 15" reflectance=".05"/>
  </asset>

  <asset>
    <texture
      type="skybox"
      builtin="gradient"
      rgb1="0.2 0.2 0.9"
      rgb2="0.1 0.6 0.2"
      width="512"
      height="3072"
    />
    <material name="stairs_geom" rgba="0.70 0.45 0.20 1" />
    <material name="stump_geom" rgba="0.35 0.24 0.12 1" />
  </asset>

  <worldbody>
    <light
      name="sun"
      directional="true"
      dir="-0.5 -0.4 -1"
      diffuse="0.4 0.4 0.4"
      specular="0.2 0.2 0.2"
      ambient="0.01 0.01 0.01"
      castshadow="true"
    />

    <light
      name="spotlight1"
      mode="targetbodycom"
      target="base"
      diffuse="0.2 0.2 0.2"
      specular="0.1 0.1 0.1"
      pos="0 -10 4"
      cutoff="10"
    />

    <light
      name="spotlight2"
      mode="targetbodycom"
      directional="true"
      target="base"
      diffuse="0.2 0.2 0.2"
      specular="0.1 0.1 0.1"
      pos="10 0 4"
      dir="1 0 0"
      cutoff="10"
      castshadow="false"
    />

    <geom name="ground" type="plane" pos="0 0 0" size="{fmt(ground_half_x)} {fmt(ground_half_y)} 0.1" material="grid"/>

{up_steps}
{pre_buffer}
{stump_geoms}
{post_buffer}
{down_steps}
  </worldbody>
</mujoco>
""",
        len(stumps),
    )


def render_generation_comment(
    platform_width: float,
    step_count: int,
    step_height: float,
    step_depth: float,
    top_platform_length: float,
    stump_zone_length: float,
    max_stump_gap: float,
    min_stump_gap: float,
    stump_height_variation: float,
    min_stump_diameter: float,
    max_stump_diameter: float,
    seed: int | None,
    generated_stump_count: int,
) -> str:
    return "\n".join(
        [
            "<!-- Generated by generate_stumps.py with arguments:",
            f"  platform-width {fmt(platform_width)}",
            f"  step-count {step_count}",
            f"  step-height {fmt(step_height)}",
            f"  step-depth {fmt(step_depth)}",
            f"  top-platform-length {fmt(top_platform_length)}",
            f"  stump-zone-length {fmt(stump_zone_length)}",
            f"  max-stump-gap {fmt(max_stump_gap)}",
            f"  min-stump-gap {fmt(min_stump_gap)}",
            f"  stump-height-variation {fmt(stump_height_variation)}",
            f"  min-stump-diameter {fmt(min_stump_diameter)}",
            f"  max-stump-diameter {fmt(max_stump_diameter)}",
            f"  seed {'none' if seed is None else seed}",
            f"  generated-stump-count {generated_stump_count}",
            "-->",
        ]
    )


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.max_stump_diameter < args.min_stump_diameter:
        parser.error("--max-stump-diameter must be greater than or equal to --min-stump-diameter")
    if args.min_stump_gap > args.max_stump_gap:
        parser.error("--min-stump-gap must be less than or equal to --max-stump-gap")

    try:
        scene_xml, generated_stump_count = render_scene(
            platform_width=args.platform_width,
            step_count=args.step_count,
            step_height=args.step_height,
            step_depth=args.step_depth,
            top_platform_length=args.top_platform_length,
            stump_zone_length=args.stump_zone_length,
            max_stump_gap=args.max_stump_gap,
            min_stump_gap=args.min_stump_gap,
            stump_height_variation=args.stump_height_variation,
            min_stump_diameter=args.min_stump_diameter,
            max_stump_diameter=args.max_stump_diameter,
            seed=args.seed,
        )
    except RuntimeError as exc:
        parser.error(str(exc))

    generation_comment = render_generation_comment(
        platform_width=args.platform_width,
        step_count=args.step_count,
        step_height=args.step_height,
        step_depth=args.step_depth,
        top_platform_length=args.top_platform_length,
        stump_zone_length=args.stump_zone_length,
        max_stump_gap=args.max_stump_gap,
        min_stump_gap=args.min_stump_gap,
        stump_height_variation=args.stump_height_variation,
        min_stump_diameter=args.min_stump_diameter,
        max_stump_diameter=args.max_stump_diameter,
        seed=args.seed,
        generated_stump_count=generated_stump_count,
    )
    xml = f"{generation_comment}\n{scene_xml}"

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_OUTPUT.write_text(xml, encoding="utf-8")
    print(f"Generated {DEFAULT_OUTPUT}")


if __name__ == "__main__":
    main()
