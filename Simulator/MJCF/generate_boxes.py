#!/usr/bin/env python3
"""Generate an MJCF scene with a sequence of boxes of varying heights."""

from __future__ import annotations

import argparse
from pathlib import Path


DEFAULT_OUTPUT = Path(__file__).with_name("boxes.xml")
FIRST_BOX_CENTER_X = 0.6
BOX_HALF_SIZE_X = 0.2
BOX_HALF_SIZE_Y = 1.0


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
            "Generate an MJCF scene with boxes of linearly changing heights. "
            "The first box center is fixed at x = 0.6 m."
        )
    )
    parser.add_argument(
        "--first-box-height",
        required=True,
        type=positive_float,
        help="Height of the first box in meters.",
    )
    parser.add_argument(
        "--last-box-height",
        required=True,
        type=positive_float,
        help="Height of the last box in meters.",
    )
    parser.add_argument(
        "--box-count",
        required=True,
        type=positive_int,
        help="Number of boxes.",
    )
    parser.add_argument(
        "--center-spacing",
        required=True,
        type=positive_float,
        help="Distance between centers of adjacent boxes in meters.",
    )
    return parser


def make_box_geoms(
    first_box_height: float,
    last_box_height: float,
    box_count: int,
    center_spacing: float,
) -> tuple[str, float, float, float]:
    if box_count == 1:
        heights = [first_box_height]
    else:
        step = (last_box_height - first_box_height) / (box_count - 1)
        heights = [first_box_height + index * step for index in range(box_count)]

    first_center_x = FIRST_BOX_CENTER_X
    last_center_x = FIRST_BOX_CENTER_X + (box_count - 1) * center_spacing
    center_x = (first_center_x + last_center_x) / 2.0
    max_height = max(heights)

    geom_lines: list[str] = []
    for index, height in enumerate(heights):
        half_height = height / 2.0
        center_x_i = FIRST_BOX_CENTER_X + index * center_spacing
        geom_lines.extend(
            [
                "    <geom",
                f'      name="box_{index + 1}"',
                '      type="box"',
                f'      pos="{fmt(center_x_i)} 0 {fmt(half_height)}"',
                f'      size="{fmt(BOX_HALF_SIZE_X)} {fmt(BOX_HALF_SIZE_Y)} {fmt(half_height)}"',
                '      contype="1"',
                '      conaffinity="1"',
                '      material="boxes_geom"',
                "    />",
            ]
        )

    return "\n".join(geom_lines), center_x, last_center_x, max_height


def render_scene(
    first_box_height: float,
    last_box_height: float,
    box_count: int,
    center_spacing: float,
) -> str:
    box_geoms, center_x, last_center_x, max_height = make_box_geoms(
        first_box_height=first_box_height,
        last_box_height=last_box_height,
        box_count=box_count,
        center_spacing=center_spacing,
    )

    ground_half_x = max(15.0, last_center_x + BOX_HALF_SIZE_X + 2.0)
    ground_half_y = 15.0
    statistic_center_z = max(0.1, max_height)
    # statistic_extent = max(11.0, last_center_x + BOX_HALF_SIZE_X + 0.4)
    statistic_extent = 2

    return f"""<mujoco model="{{name}} boxes scene">
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
    <material name="boxes_geom" rgba="0.70 0.45 0.20 1" />
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

{box_geoms}
  </worldbody>
</mujoco>
"""


def render_generation_comment(
    first_box_height: float,
    last_box_height: float,
    box_count: int,
    center_spacing: float,
) -> str:
    return "\n".join(
        [
            "<!-- Generated by generate_boxes.py with arguments:",
            f"  first-box-height {fmt(first_box_height)}",
            f"  last-box-height {fmt(last_box_height)}",
            f"  box-count {box_count}",
            f"  center-spacing {fmt(center_spacing)}",
            "-->",
        ]
    )


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.box_count == 1 and abs(args.first_box_height - args.last_box_height) > 1e-12:
        parser.error("for --box-count 1, --first-box-height and --last-box-height must match")

    scene_xml = render_scene(
        first_box_height=args.first_box_height,
        last_box_height=args.last_box_height,
        box_count=args.box_count,
        center_spacing=args.center_spacing,
    )
    generation_comment = render_generation_comment(
        first_box_height=args.first_box_height,
        last_box_height=args.last_box_height,
        box_count=args.box_count,
        center_spacing=args.center_spacing,
    )
    xml = f"{generation_comment}\n{scene_xml}"

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_OUTPUT.write_text(xml, encoding="utf-8")
    print(f"Generated {DEFAULT_OUTPUT}")


if __name__ == "__main__":
    main()
