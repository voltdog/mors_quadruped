#!/usr/bin/env python3
"""Generate an MJCF scene with scattered free-falling boards."""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path


DEFAULT_OUTPUT = Path(__file__).with_name("boards.xml")
DEFAULT_START_X = 0.8
DEFAULT_START_HEIGHT = 0.5
DEFAULT_DENSITY = 500.0
DEFAULT_LENGTH_VARIATION = 0.35
DEFAULT_THICKNESS_VARIATION = 0.25
DEFAULT_WIDTH_VARIATION = 0.30
MIN_DIMENSION = 0.005
MATERIAL_PRESETS = (
    {
        "name": "board_wood",
        "rgba": "0.70 0.45 0.20 1",
        "density_scale": 1.0,
        "friction": (1.0, 0.02, 0.01),
    },
    {
        "name": "board_rubber",
        "rgba": "0.15 0.15 0.15 1",
        "density_scale": 1.35,
        "friction": (1.35, 0.03, 0.02),
    },
    {
        "name": "board_metal",
        "rgba": "0.63 0.66 0.72 1",
        "density_scale": 1.8,
        "friction": (0.75, 0.015, 0.008),
    },
)


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


def fmt_values(values: tuple[float, ...]) -> str:
    return " ".join(fmt(value) for value in values)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an MJCF scene with randomly sized dynamic boards "
            "distributed uniformly along the robot path on the X axis."
        )
    )
    parser.add_argument(
        "--board-count",
        required=True,
        type=positive_int,
        help="Number of boards.",
    )
    parser.add_argument(
        "--mean-length",
        required=True,
        type=positive_float,
        help="Average board length in meters.",
    )
    parser.add_argument(
        "--mean-thickness",
        required=True,
        type=positive_float,
        help="Average board thickness in meters.",
    )
    parser.add_argument(
        "--mean-width",
        required=True,
        type=positive_float,
        help="Average board width in meters.",
    )
    parser.add_argument(
        "--length-variation",
        default=DEFAULT_LENGTH_VARIATION,
        type=nonnegative_float,
        help=(
            "Relative half-range for random board length. "
            "0.35 means sampling from 65%% to 135%% of the average."
        ),
    )
    parser.add_argument(
        "--thickness-variation",
        default=DEFAULT_THICKNESS_VARIATION,
        type=nonnegative_float,
        help=(
            "Relative half-range for random board thickness. "
            "0.25 means sampling from 75%% to 125%% of the average."
        ),
    )
    parser.add_argument(
        "--width-variation",
        default=DEFAULT_WIDTH_VARIATION,
        type=nonnegative_float,
        help=(
            "Relative half-range for random board width. "
            "0.30 means sampling from 70%% to 130%% of the average."
        ),
    )
    parser.add_argument(
        "--start-x",
        default=DEFAULT_START_X,
        type=float,
        help="X coordinate of the first board center in meters. Default: 0.8.",
    )
    spacing_group = parser.add_mutually_exclusive_group()
    spacing_group.add_argument(
        "--x-spacing",
        type=positive_float,
        help="Fixed spacing between board centers along X in meters.",
    )
    spacing_group.add_argument(
        "--x-span",
        type=nonnegative_float,
        help=(
            "Distance between the centers of the first and the last board along X "
            "in meters."
        ),
    )
    parser.add_argument(
        "--y-span",
        type=positive_float,
        help="Total lateral span for board placement in meters.",
    )
    parser.add_argument(
        "--start-height",
        default=DEFAULT_START_HEIGHT,
        type=nonnegative_float,
        help=(
            "Initial clearance from the lowest board point to the ground in meters. "
            "Default: 0.5."
        ),
    )
    parser.add_argument(
        "--density",
        default=DEFAULT_DENSITY,
        type=positive_float,
        help="Board density in kg/m^3. Default: 500.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="Random seed for reproducible generation.",
    )
    parser.add_argument(
        "--mixed-materials",
        action="store_true",
        help=(
            "Assign a random material preset to each board with different color, "
            "density and friction."
        ),
    )
    return parser


def sample_dimension(
    rng: random.Random,
    mean: float,
    variation: float,
) -> float:
    lower = max(MIN_DIMENSION, mean * (1.0 - variation))
    upper = max(lower, mean * (1.0 + variation))
    return rng.uniform(lower, upper)


def quaternion_to_matrix(quat: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    w, x, y, z = quat
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def random_quaternion(rng: random.Random) -> tuple[float, float, float, float]:
    u1 = rng.random()
    u2 = rng.random()
    u3 = rng.random()
    x = math.sqrt(1.0 - u1) * math.sin(2.0 * math.pi * u2)
    y = math.sqrt(1.0 - u1) * math.cos(2.0 * math.pi * u2)
    z = math.sqrt(u1) * math.sin(2.0 * math.pi * u3)
    w = math.sqrt(u1) * math.cos(2.0 * math.pi * u3)
    return (w, x, y, z)


def vertical_half_extent(
    half_sizes: tuple[float, float, float],
    rotation: tuple[tuple[float, float, float], ...],
) -> float:
    hx, hy, hz = half_sizes
    return (
        abs(rotation[2][0]) * hx
        + abs(rotation[2][1]) * hy
        + abs(rotation[2][2]) * hz
    )


def axis_half_extent(
    half_sizes: tuple[float, float, float],
    rotation: tuple[tuple[float, float, float], ...],
    axis_index: int,
) -> float:
    hx, hy, hz = half_sizes
    return (
        abs(rotation[axis_index][0]) * hx
        + abs(rotation[axis_index][1]) * hy
        + abs(rotation[axis_index][2]) * hz
    )


def board_inertia(
    half_sizes: tuple[float, float, float],
    density: float,
) -> tuple[float, tuple[float, float, float]]:
    hx, hy, hz = half_sizes
    length = 2.0 * hx
    width = 2.0 * hy
    thickness = 2.0 * hz
    mass = density * length * width * thickness
    ixx = (mass / 12.0) * (width * width + thickness * thickness)
    iyy = (mass / 12.0) * (length * length + thickness * thickness)
    izz = (mass / 12.0) * (length * length + width * width)
    return mass, (ixx, iyy, izz)


def automatic_spacing(
    mean_length: float,
    mean_thickness: float,
    mean_width: float,
) -> float:
    mean_diagonal = math.sqrt(
        mean_length * mean_length
        + mean_width * mean_width
        + mean_thickness * mean_thickness
    )
    return max(0.35, mean_diagonal * 1.2)


def automatic_y_span(mean_width: float) -> float:
    return max(1.2, mean_width * 3.0)


def resolve_x_spacing(
    board_count: int,
    mean_length: float,
    mean_thickness: float,
    mean_width: float,
    x_spacing: float | None,
    x_span: float | None,
) -> float:
    if x_spacing is not None:
        return x_spacing
    if x_span is not None:
        if board_count == 1:
            return 0.0
        return x_span / (board_count - 1)
    return automatic_spacing(
        mean_length=mean_length,
        mean_thickness=mean_thickness,
        mean_width=mean_width,
    )


def make_y_positions(
    board_count: int,
    y_span: float,
    rng: random.Random,
) -> list[float]:
    if board_count == 1:
        return [0.0]

    cell_size = y_span / board_count
    positions = []
    for index in range(board_count):
        base = -0.5 * y_span + (index + 0.5) * cell_size
        jitter = rng.uniform(-0.25 * cell_size, 0.25 * cell_size)
        positions.append(base + jitter)
    rng.shuffle(positions)
    return positions


def make_board_bodies(
    board_count: int,
    mean_length: float,
    mean_thickness: float,
    mean_width: float,
    length_variation: float,
    thickness_variation: float,
    width_variation: float,
    start_x: float,
    x_spacing: float,
    y_span: float,
    start_height: float,
    density: float,
    mixed_materials: bool,
    rng: random.Random,
) -> tuple[str, float, float, float, float]:
    bodies: list[str] = []
    y_positions = make_y_positions(board_count=board_count, y_span=y_span, rng=rng)

    max_center_z = start_height
    max_reach_x = start_x
    max_reach_y = y_span / 2.0

    for index in range(board_count):
        length = sample_dimension(rng, mean_length, length_variation)
        thickness = sample_dimension(rng, mean_thickness, thickness_variation)
        width = sample_dimension(rng, mean_width, width_variation)
        half_sizes = (length / 2.0, width / 2.0, thickness / 2.0)

        quat = random_quaternion(rng)
        rotation = quaternion_to_matrix(quat)
        x_half_extent = axis_half_extent(half_sizes, rotation, axis_index=0)
        y_half_extent = axis_half_extent(half_sizes, rotation, axis_index=1)
        z_half_extent = vertical_half_extent(half_sizes, rotation)
        x = start_x + index * x_spacing
        y = y_positions[index]
        z = start_height + z_half_extent

        material_name = "board_geom"
        friction_attr = ""
        board_density = density
        if mixed_materials:
            material = rng.choice(MATERIAL_PRESETS)
            material_name = material["name"]
            board_density = density * material["density_scale"]
            friction_attr = f'        friction="{fmt_values(material["friction"])}"'

        mass, inertia = board_inertia(half_sizes, board_density)
        geom_lines = [
            "      <geom",
            '        type="box"',
            f'        size="{fmt_values(half_sizes)}"',
            '        contype="1"',
            '        conaffinity="1"',
        ]
        if friction_attr:
            geom_lines.append(friction_attr)
        geom_lines.extend(
            [
                f'        material="{material_name}"',
                "      />",
            ]
        )

        bodies.extend(
            [
                f'    <body name="board_{index + 1}" pos="{fmt(x)} {fmt(y)} {fmt(z)}" quat="{fmt_values(quat)}">',
                "      <freejoint/>",
                f'      <inertial pos="0 0 0" mass="{fmt(mass)}" diaginertia="{fmt_values(inertia)}" />',
                *geom_lines,
                "    </body>",
            ]
        )

        max_center_z = max(max_center_z, z + z_half_extent)
        max_reach_x = max(max_reach_x, x + x_half_extent)
        max_reach_y = max(max_reach_y, abs(y) + y_half_extent)

    scene_center_x = start_x + (board_count - 1) * x_spacing / 2.0
    return "\n".join(bodies), max_reach_x, max_reach_y, max_center_z, scene_center_x


def render_material_assets(mixed_materials: bool) -> str:
    if not mixed_materials:
        return '    <material name="board_geom" rgba="0.70 0.45 0.20 1" />'

    return "\n".join(
        f'    <material name="{preset["name"]}" rgba="{preset["rgba"]}" />'
        for preset in MATERIAL_PRESETS
    )


def render_scene(
    board_count: int,
    mean_length: float,
    mean_thickness: float,
    mean_width: float,
    length_variation: float,
    thickness_variation: float,
    width_variation: float,
    start_x: float,
    x_spacing: float | None,
    x_span: float | None,
    y_span: float | None,
    start_height: float,
    density: float,
    mixed_materials: bool,
    rng: random.Random,
) -> str:
    resolved_x_spacing = resolve_x_spacing(
        board_count=board_count,
        mean_length=mean_length,
        mean_thickness=mean_thickness,
        mean_width=mean_width,
        x_spacing=x_spacing,
        x_span=x_span,
    )
    resolved_y_span = y_span or automatic_y_span(mean_width)
    material_assets = render_material_assets(mixed_materials)

    board_bodies, max_reach_x, max_reach_y, max_center_z, scene_center_x = make_board_bodies(
        board_count=board_count,
        mean_length=mean_length,
        mean_thickness=mean_thickness,
        mean_width=mean_width,
        length_variation=length_variation,
        thickness_variation=thickness_variation,
        width_variation=width_variation,
        start_x=start_x,
        x_spacing=resolved_x_spacing,
        y_span=resolved_y_span,
        start_height=start_height,
        density=density,
        mixed_materials=mixed_materials,
        rng=rng,
    )

    ground_half_x = max(15.0, max_reach_x + resolved_x_spacing + 2.0)
    ground_half_y = max(15.0, max_reach_y + 2.0)
    statistic_center_z = max(0.2, max_center_z / 2.0)
    statistic_extent = max(
        ground_half_x,
        ground_half_y,
        max_center_z + 2.0,
    )

    return f"""<mujoco model="{{name}} boards scene">
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
{material_assets}
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

{board_bodies}
  </worldbody>
</mujoco>
"""


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    rng = random.Random(args.seed)

    xml = render_scene(
        board_count=args.board_count,
        mean_length=args.mean_length,
        mean_thickness=args.mean_thickness,
        mean_width=args.mean_width,
        length_variation=args.length_variation,
        thickness_variation=args.thickness_variation,
        width_variation=args.width_variation,
        start_x=args.start_x,
        x_spacing=args.x_spacing,
        x_span=args.x_span,
        y_span=args.y_span,
        start_height=args.start_height,
        density=args.density,
        mixed_materials=args.mixed_materials,
        rng=rng,
    )

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_OUTPUT.write_text(xml, encoding="utf-8")
    print(f"Generated {DEFAULT_OUTPUT}")


if __name__ == "__main__":
    main()
