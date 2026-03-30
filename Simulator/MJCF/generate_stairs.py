#!/usr/bin/env python3
"""Generate an MJCF scene with a stepped pyramid."""

from __future__ import annotations

import argparse
from pathlib import Path


FIRST_STEP_START_X = 0.5
DEFAULT_OUTPUT = Path(__file__).with_name("stairs.xml")


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text else "0"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a stepped-pyramid MJCF scene. "
            "The first step starts at x = 0.5 m."
        )
    )
    parser.add_argument(
        "--step-height",
        required=True,
        type=positive_float,
        help="Height of one step in meters.",
    )
    parser.add_argument(
        "--step-length",
        required=True,
        type=positive_float,
        help="Horizontal step inset in meters.",
    )
    parser.add_argument(
        "--step-count",
        required=True,
        type=positive_int,
        help="Number of steps in the pyramid.",
    )
    parser.add_argument(
        "--top-platform-size",
        required=True,
        type=positive_float,
        help="Side length of the square top platform in meters.",
    )
    return parser


def make_step_geoms(
    step_height: float,
    step_length: float,
    step_count: int,
    top_platform_size: float,
) -> tuple[str, float, float]:
    bottom_side = top_platform_size + 2.0 * step_length * (step_count - 1)
    center_x = FIRST_STEP_START_X + bottom_side / 2.0

    geom_lines: list[str] = []
    for level in range(step_count):
        remaining_rings = step_count - level - 1
        side = top_platform_size + 2.0 * step_length * remaining_rings
        top_height = step_height * (level + 1)
        geom_lines.extend(
            [
                "    <geom",
                f'      name="step_{level + 1}"',
                '      type="box"',
                f'      pos="{fmt(center_x)} 0 {fmt(top_height / 2.0)}"',
                f'      size="{fmt(side / 2.0)} {fmt(side / 2.0)} {fmt(top_height / 2.0)}"',
                '      contype="1"',
                '      conaffinity="1"',
                '      material="stairs_geom"',
                "    />",
            ]
        )

    return "\n".join(geom_lines), bottom_side, center_x


def render_scene(
    step_height: float,
    step_length: float,
    step_count: int,
    top_platform_size: float,
) -> str:
    step_geoms, bottom_side, center_x = make_step_geoms(
        step_height=step_height,
        step_length=step_length,
        step_count=step_count,
        top_platform_size=top_platform_size,
    )

    pyramid_height = step_height * step_count
    ground_half_x = max(15.0, center_x + bottom_side / 2.0 + 2.0)
    ground_half_y = max(15.0, bottom_side / 2.0 + 2.0)
    statistic_center_z = max(0.1, pyramid_height / 2.0)
    statistic_extent = max(
        ground_half_x,
        ground_half_y,
        pyramid_height + 2.0,
    )

    return f"""<mujoco model="{{name}} stairs scene">
  <include file="{{path}}" />

  <statistic center="{fmt(center_x)} 0 {fmt(statistic_center_z)}" extent="{fmt(statistic_extent)}" meansize="0.04" />

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

{step_geoms}
  </worldbody>
</mujoco>
"""


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    xml = render_scene(
        step_height=args.step_height,
        step_length=args.step_length,
        step_count=args.step_count,
        top_platform_size=args.top_platform_size,
    )

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_OUTPUT.write_text(xml, encoding="utf-8")
    print(f"Generated {DEFAULT_OUTPUT}")


if __name__ == "__main__":
    main()
