#!/usr/bin/env python3
"""Generate an MJCF scene with a trapezoidal ramp."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


FIRST_RAMP_START_X = 0.5
DEFAULT_OUTPUT = Path(__file__).with_name("ramp.xml")
RAMP_WIDTH = 2.0
SURFACE_THICKNESS = 0.04


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def slope_angle(value: str) -> float:
    parsed = float(value)
    if parsed <= 0 or parsed >= 89:
        raise argparse.ArgumentTypeError("angle must be between 0 and 89 degrees")
    return parsed


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text else "0"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an MJCF trapezoidal-ramp scene. "
            "The ramp starts at x = 0.5 m."
        )
    )
    parser.add_argument(
        "--slope-angle",
        required=True,
        type=slope_angle,
        help="Slope angle in degrees.",
    )
    parser.add_argument(
        "--height",
        required=True,
        type=positive_float,
        help="Ramp height in meters.",
    )
    parser.add_argument(
        "--flat-length",
        required=True,
        type=positive_float,
        help="Length of the flat top section in meters.",
    )
    return parser


def make_ramp_geoms(
    slope_angle_deg: float,
    height: float,
    flat_length: float,
) -> tuple[str, float, float]:
    slope_angle_rad = math.radians(slope_angle_deg)
    slope_run = height / math.tan(slope_angle_rad)
    slope_length = math.hypot(slope_run, height)
    half_width = RAMP_WIDTH / 2.0
    half_thickness = SURFACE_THICKNESS / 2.0

    up_surface_mid_x = FIRST_RAMP_START_X + slope_run / 2.0
    up_surface_mid_z = height / 2.0
    up_normal_x = -math.sin(slope_angle_rad)
    up_normal_z = math.cos(slope_angle_rad)
    up_center_x = up_surface_mid_x - up_normal_x * half_thickness
    up_center_z = up_surface_mid_z - up_normal_z * half_thickness

    top_start_x = FIRST_RAMP_START_X + slope_run
    top_center_x = top_start_x + flat_length / 2.0
    top_center_z = height - half_thickness

    down_surface_mid_x = top_start_x + flat_length + slope_run / 2.0
    down_surface_mid_z = height / 2.0
    down_normal_x = math.sin(slope_angle_rad)
    down_normal_z = math.cos(slope_angle_rad)
    down_center_x = down_surface_mid_x - down_normal_x * half_thickness
    down_center_z = down_surface_mid_z - down_normal_z * half_thickness

    end_x = FIRST_RAMP_START_X + 2.0 * slope_run + flat_length
    scene_center_x = (FIRST_RAMP_START_X + end_x) / 2.0

    geom_lines = [
        "    <geom",
        '      name="ramp_up"',
        '      type="box"',
        f'      pos="{fmt(up_center_x)} 0 {fmt(up_center_z)}"',
        f'      size="{fmt(slope_length / 2.0)} {fmt(half_width)} {fmt(half_thickness)}"',
        f'      euler="0 -{fmt(slope_angle_deg)} 0"',
        '      contype="1"',
        '      conaffinity="1"',
        '      material="ramp_geom"',
        "    />",
        "    <geom",
        '      name="ramp_top"',
        '      type="box"',
        f'      pos="{fmt(top_center_x)} 0 {fmt(top_center_z)}"',
        f'      size="{fmt(flat_length / 2.0)} {fmt(half_width)} {fmt(half_thickness)}"',
        '      contype="1"',
        '      conaffinity="1"',
        '      material="ramp_geom"',
        "    />",
        "    <geom",
        '      name="ramp_down"',
        '      type="box"',
        f'      pos="{fmt(down_center_x)} 0 {fmt(down_center_z)}"',
        f'      size="{fmt(slope_length / 2.0)} {fmt(half_width)} {fmt(half_thickness)}"',
        f'      euler="0 {fmt(slope_angle_deg)} 0"',
        '      contype="1"',
        '      conaffinity="1"',
        '      material="ramp_geom"',
        "    />",
    ]

    return "\n".join(geom_lines), end_x, scene_center_x


def render_scene(
    slope_angle_deg: float,
    height: float,
    flat_length: float,
) -> str:
    ramp_geoms, end_x, scene_center_x = make_ramp_geoms(
        slope_angle_deg=slope_angle_deg,
        height=height,
        flat_length=flat_length,
    )

    ground_half_x = max(15.0, end_x + 2.0)
    ground_half_y = max(15.0, RAMP_WIDTH / 2.0 + 2.0)
    statistic_center_z = max(0.1, height / 2.0)
    statistic_extent = max(
        ground_half_x,
        ground_half_y,
        height + 2.0,
    )

    return f"""<mujoco model="{{name}} ramp scene">
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
    <material name="ramp_geom" rgba="0.55 0.40 0.20 1" />
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

{ramp_geoms}
  </worldbody>
</mujoco>
"""


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    xml = render_scene(
        slope_angle_deg=args.slope_angle,
        height=args.height,
        flat_length=args.flat_length,
    )

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_OUTPUT.write_text(xml, encoding="utf-8")
    print(f"Generated {DEFAULT_OUTPUT}")


if __name__ == "__main__":
    main()
