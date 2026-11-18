"""Panda bridge-building rehearsal using Robotics Toolbox and Swift."""

from __future__ import annotations

import argparse
import html
import json
from dataclasses import dataclass
from itertools import product
from pathlib import Path

import numpy as np
from spatialmath import SE3

import roboticstoolbox as rtb


@dataclass
class Block:
    """A rectangular block with supply and placement poses."""

    name: str
    size: np.ndarray
    source: SE3
    target: SE3
    yaw: float
    color: tuple[float, float, float, float]
    grasp_x: float = 0.0
    side_approach: float = 0.0


@dataclass
class Scene:
    """Validated scene dimensions in metres and radians."""

    blocks: list[Block]
    banks: list[tuple[np.ndarray, SE3]]
    safe_z: float
    speed: float
    angular_speed: float
    opening: float
    margin: float
    observe: float
    example: bool


def load_scene(path: Path) -> Scene:
    """Load laboratory dimensions and reject incompatible geometry.

    :param path: JSON configuration, with lengths explicitly in millimetres.
    :returns: Validated scene in SI units.
    """
    config = json.loads(path.read_text(encoding="utf-8"))
    layout = config.get("layout", "cross_stack")
    if layout not in ("bank_towers", "bank_pads", "cross_stack"):
        raise ValueError("layout must be bank_towers, bank_pads or cross_stack")

    def vector(key: str, shape: tuple[int, ...]) -> np.ndarray:
        value = np.asarray(config[key], dtype=float) / 1000
        if value.shape != shape or not np.all(np.isfinite(value)):
            raise ValueError(f"{key}: expected finite values with shape {shape}")
        return value

    pier = vector("pier_block_mm", (3,))
    deck = vector("deck_block_mm", (3,))
    centre = vector("pier_centre_mm", (2,))
    bank_xy = vector("bank_size_mm", (2,))
    supply = vector("supply_xy_mm", (8, 2))
    scalars = {}
    for key in [
        "pier_gap_mm",
        "bank_inner_gap_mm",
        "deck_joint_gap_mm",
        "safe_tcp_z_mm",
        "linear_speed_mm_s",
        "angular_speed_deg_s",
        "gripper_opening_mm",
        "gripper_margin_mm",
        "observe_seconds",
    ]:
        value = float(config[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
        scalars[key] = value
    if np.any(pier <= 0) or np.any(deck <= 0) or np.any(bank_xy <= 0):
        raise ValueError("All block and bank dimensions must be positive")
    opening = scalars["gripper_opening_mm"] / 1000
    margin = scalars["gripper_margin_mm"] / 1000
    if opening > 0.080 or max(pier[1], deck[1]) + margin > opening:
        raise ValueError(
            "Panda opening is at most 80 mm; block width + margin must fit"
        )
    gap = scalars["pier_gap_mm"] / 1000
    inner_gap = scalars["bank_inner_gap_mm"] / 1000
    joint_gap = scalars["deck_joint_gap_mm"] / 1000
    if layout == "cross_stack" and pier[0] <= 2 * pier[1] + gap:
        raise ValueError(
            "Cross-stacked layers need positive overlap at all four crossings"
        )
    if layout == "cross_stack" and inner_gap <= pier[0]:
        raise ValueError("Riverbanks must leave room for the central pier")
    bank_height = float(config.get("bank_height_mm", 3 * pier[2] * 1000)) / 1000
    if not np.isfinite(bank_height) or bank_height <= 0:
        raise ValueError("bank_height_mm must be finite and positive")
    if layout == "cross_stack" and not np.isclose(
        bank_height, 3 * pier[2], atol=0.0001
    ):
        raise ValueError(
            f"Riverbanks are {bank_height * 1000:g} mm high, but the three-layer pier is "
            f"{3 * pier[2] * 1000:g} mm high. Resolve the height difference before planning."
        )
    deck_base = bank_height if layout == "cross_stack" else bank_height + pier[2]
    tower_spacing = float(config.get("tower_column_spacing_mm", 44)) / 1000
    grasp_offset = float(config.get("tower_grasp_offset_mm", 20)) / 1000
    standoff = float(config.get("tower_standoff_mm", 80)) / 1000
    if layout == "bank_towers":
        values = np.array([tower_spacing, grasp_offset, standoff])
        if not np.all(np.isfinite(values)) or np.any(values <= 0):
            raise ValueError(
                "Tower spacing, grasp offset and standoff must be positive"
            )
        if tower_spacing - pier[1] < margin / 2 + 0.010 + 0.002:
            raise ValueError("Tower columns need clearance for the gripper fingers")
        if tower_spacing + pier[1] > min(pier[0], bank_xy[1]):
            raise ValueError("Both columns must fit under their cap and on the bank")
        if grasp_offset >= pier[0] / 2 - 0.010:
            raise ValueError("Column grasp needs at least 10 mm clearance from its end")
        if standoff < 0.050 or pier[1] > bank_xy[0]:
            raise ValueError("Tower side approach needs at least 50 mm standoff")
        # Caps lie across y; require positive bearing length at both deck ends.
        tower_x = (inner_gap + pier[1]) / 2
        if deck[0] / 2 - (tower_x - pier[1] / 2) < 0.015:
            raise ValueError("Deck needs at least 15 mm bearing on each tower cap")
        if deck[1] > pier[0] or deck[1] < tower_spacing:
            raise ValueError("Deck width must cover both columns and fit on the cap")
        deck_base = bank_height + pier[0] + pier[2]
    spacing = float(config.get("pad_spacing_mm", 40)) / 1000
    if layout == "bank_pads":
        # Allow 10 mm finger thickness and 2 mm placement clearance per side.
        needed_gap = margin / 2 + 0.010 + 0.002
        if (
            not np.isfinite(spacing)
            or spacing - pier[1] < needed_gap
            or joint_gap < needed_gap
        ):
            raise ValueError(
                "Pad/deck gaps are too small for the assumed 10 mm fingers and 2 mm clearance"
            )
        if pier[0] > bank_xy[0] or 2 * spacing + pier[1] > bank_xy[1]:
            raise ValueError("All three pads must fit completely on each riverbank")
        if inner_gap >= deck[0] - 0.020:
            raise ValueError("Decks need at least 10 mm bearing length on each bank")
        if (deck[0] - inner_gap) / 2 > pier[0]:
            raise ValueError("Deck extends past the bearing pads")
        if 2 * deck[1] + joint_gap > bank_xy[1]:
            raise ValueError(
                "Parallel deck width and gap must fit across the riverbanks"
            )
        deck_centre_y = (deck[1] + joint_gap) / 2
        if abs(deck_centre_y - spacing) > pier[1] / 2 - 0.005:
            raise ValueError(
                "Deck centreline needs at least 5 mm lateral support margin on its outer pad"
            )
    banks = []
    for sign in (-1, 1):
        xy = centre + [sign * (inner_gap + bank_xy[0]) / 2, 0]
        banks.append((np.r_[bank_xy, bank_height], SE3(*xy, bank_height / 2)))
    blocks = []
    colors = [
        (0.30, 0.60, 0.85, 1),
        (0.90, 0.65, 0.25, 1),
        (0.25, 0.70, 0.75, 1),
        (0.65, 0.50, 0.80, 1),
    ]
    for i in range(8):
        grasp_x, side_approach = 0.0, 0.0
        rotation = SE3()
        if layout == "bank_towers":
            yaw = 0.0
            if i < 4:
                bank, column = divmod(i, 2)
                sign = 2 * bank - 1
                xy = centre + [
                    sign * (inner_gap + pier[1]) / 2,
                    (1 - 2 * column) * tower_spacing / 2,
                ]
                size, z = pier, bank_height + pier[0] / 2
                rotation = SE3.Rz(np.pi / 2) * SE3.Ry(-np.pi / 2)
                grasp_x, side_approach = grasp_offset, standoff
                name, color = f"Tower {bank + 1} upright {column + 1}", colors[bank]
            elif i < 6:
                sign = 2 * (i - 4) - 1
                xy = centre + [sign * (inner_gap + pier[1]) / 2, 0]
                size, z = pier, bank_height + pier[0] + pier[2] / 2
                yaw, rotation = np.pi / 2, SE3.Rz(np.pi / 2)
                name, color = f"Tower {i - 3} cap", colors[2]
            else:
                xy = centre
                size, z = deck, deck_base + (i - 6 + 0.5) * deck[2]
                name, color = f"Deck layer {i - 5}", colors[3]
        elif i < 6:
            if layout == "bank_pads":
                bank, slot = divmod(i, 3)
                xy = centre + [
                    (2 * bank - 1) * (inner_gap + pier[0]) / 2,
                    (slot - 1) * spacing,
                ]
                size, z, yaw = pier, bank_height + pier[2] / 2, 0.0
                name, color = f"Bank {bank + 1} pad {slot + 1}", colors[bank]
            else:
                layer, slot = divmod(i, 2)
                yaw = np.pi / 2 if layer == 1 else 0.0
                offset = (2 * slot - 1) * (pier[1] + gap) / 2
                xy = centre + ([offset, 0] if layer == 1 else [0, offset])
                size, z = pier, (layer + 0.5) * pier[2]
                name = f"Pier {layer + 1}.{slot + 1}"
                color = colors[layer]
        else:
            yaw = 0.0
            sign = 2 * (i - 6) - 1
            xy = centre + (
                [0, sign * (deck[1] + joint_gap) / 2]
                if layout == "bank_pads"
                else [sign * (deck[0] + joint_gap) / 2, 0]
            )
            size, z = deck, deck_base + deck[2] / 2
            name, color = f"Deck {i - 5}", colors[3]
        if layout != "bank_towers":
            rotation = SE3.Rz(yaw)
        blocks.append(
            Block(
                name,
                size.copy(),
                SE3(*supply[i], size[2] / 2),
                SE3(*xy, z) * rotation,
                yaw,
                color,
                grasp_x,
                side_approach,
            )
        )
    # Supply footprints must be separated; all start with their long side along x.
    for i, block in enumerate(blocks):
        for other in blocks[:i]:
            distance = np.abs(block.source.t[:2] - other.source.t[:2])
            if np.all(distance < (block.size[:2] + other.size[:2]) / 2 + 0.005):
                raise ValueError(
                    f"Supply blocks overlap or lack 5 mm clearance: {block.name}, {other.name}"
                )
    pier_half = (2 * pier[1] + gap) / 2
    if layout == "cross_stack" and deck[1] / 2 <= gap / 2:
        raise ValueError("Deck must overlap both top-layer support blocks across y")
    if layout == "cross_stack" and joint_gap / 2 >= pier[0] / 2:
        raise ValueError("Deck joint leaves no overlap with the pier")
    outer_edge = deck[0] + joint_gap / 2
    if (
        layout == "cross_stack"
        and not inner_gap / 2 < outer_edge <= inner_gap / 2 + bank_xy[0]
    ):
        raise ValueError(
            "Deck length must reach its riverbank without extending beyond it"
        )
    if bank_xy[1] < deck[1] or pier_half <= 0:
        raise ValueError("Riverbank must support the deck width")
    safe_z = scalars["safe_tcp_z_mm"] / 1000
    top = deck_base + (2 if layout == "bank_towers" else 1) * deck[2]
    payload_height = pier[0] if layout == "bank_towers" else max(pier[2], deck[2])
    if safe_z < top + payload_height + 0.08:
        raise ValueError(
            "safe_tcp_z_mm needs at least 80 mm clearance above the finished structure and payload"
        )
    observe = scalars["observe_seconds"]
    if observe < 10:
        raise ValueError("The proposal requires at least ten seconds of observation")
    return Scene(
        blocks,
        banks,
        safe_z,
        scalars["linear_speed_mm_s"] / 1000,
        np.deg2rad(scalars["angular_speed_deg_s"]),
        opening,
        margin,
        observe,
        bool(config.get("example_dimensions", True)),
    )


@dataclass
class Frame:
    """A sampled joint state and discrete task state."""

    t: float
    q: np.ndarray
    opening: float
    held: int | None
    placed: int
    stage: str


def grasp_pose(block: Block, target: bool = False) -> SE3:
    """Return the TCP pose fixed to the block's chosen grasp faces.

    :param block: Block to grasp.
    :param target: Use the placement pose instead of its supply pose.
    :returns: TCP pose in the robot base frame.
    """
    pose = block.target if target else block.source
    return pose * SE3(block.grasp_x, 0, min(block.size[2] * 0.2, 0.006)) * SE3.Rx(np.pi)


def build_plan(robot: rtb.Robot, scene: Scene) -> list[Frame]:
    """Precompute checked IK and joint/task-space trajectories.

    :param robot: Panda kinematic model.
    :param scene: Validated bridge and supply arrangement.
    :returns: Timestamped trajectory with attachment and release events.
    """
    frames = [Frame(0, robot.qr.copy(), scene.opening, None, 0, "Ready")]
    velocity_limit = np.asarray(robot.qdlim[: robot.n]) * 0.35

    def append(
        qs: np.ndarray,
        duration: float,
        opening: float,
        held: int | None,
        placed: int,
        stage: str,
    ) -> None:
        count = len(qs)
        # Stretch the segment until its sampled joint speeds respect the limit.
        duration = max(
            duration,
            float(np.max(np.abs(np.diff(qs, axis=0)) / velocity_limit)) * (count - 1),
        )
        previous = frames[-1]
        frames.append(
            Frame(previous.t, previous.q.copy(), previous.opening, held, placed, stage)
        )
        for i, q in enumerate(qs[1:], 1):
            width = previous.opening + (opening - previous.opening) * i / (count - 1)
            frames.append(
                Frame(
                    previous.t + duration * i / (count - 1),
                    q.copy(),
                    width,
                    held,
                    placed,
                    stage,
                )
            )

    def move(
        goal: SE3,
        held: int | None,
        placed: int,
        stage: str,
        joint: bool = False,
        precise: bool = False,
    ) -> None:
        q0 = frames[-1].q
        start = robot.fkine(q0)
        distance = np.linalg.norm(start.t - goal.t)
        angle = np.linalg.norm((start.inv() * goal).angvec()[0])
        speed = min(scene.speed, 0.025) if precise else scene.speed
        duration = max(
            1.2 if precise else 0.7,
            1.9 * distance / speed,
            1.9 * angle / scene.angular_speed,
        )

        def solve(pose: SE3, seed_q: np.ndarray) -> np.ndarray:
            solution = robot.ikine_LM(
                pose,
                q0=seed_q,
                seed=7,
                ilimit=80,
                slimit=60 if joint else 6,
                tol=1e-11,
                joint_limits=True,
                method="sugihara",
                k=0.001,
            )
            if not solution.success:
                raise ValueError(
                    f"IK failed at {stage}, xyz={pose.t.round(3)}: {solution.reason}"
                )
            actual = robot.fkine(solution.q)
            if (
                np.linalg.norm(actual.t - pose.t) > 0.0001
                or np.linalg.norm(actual.R - pose.R) > 0.001
            ):
                raise ValueError(f"Pose tolerance failed at {stage}")
            return solution.q

        if joint:
            qs = rtb.jtraj(q0, solve(goal, q0), 60).q
        else:
            count = max(
                10,
                int(np.ceil(distance / 0.008)) + 1,
                int(np.ceil(angle / np.deg2rad(4))) + 1,
            )
            poses = rtb.ctraj(start, goal, s=rtb.quintic(0, 1, count).q)
            qs = [q0]
            for pose in poses[1:]:
                q = solve(pose, qs[-1])
                if np.max(np.abs(q - qs[-1])) > 0.45:
                    raise ValueError(
                        f"IK branch jump at {stage}: {qs[-1].round(2)} -> {q.round(2)}; change layout or safe height"
                    )
                qs.append(q)
            qs = np.asarray(qs)
        append(qs, duration, frames[-1].opening, held, placed, stage)

    def grip(width: float, held: int | None, placed: int, stage: str) -> None:
        append(
            np.repeat(frames[-1].q[None, :], 12, axis=0),
            0.8,
            width,
            held,
            placed,
            stage,
        )

    for i, block in enumerate(scene.blocks):
        print(f"Planning {i + 1}/8: {block.name}", flush=True)
        pick, place = grasp_pose(block), grasp_pose(block, True)
        approach = (
            place * SE3.Tz(-block.side_approach) if block.side_approach else place
        )
        above_pick, above_place = SE3(pick.A.copy()), SE3(approach.A.copy())
        above_pick.t[2] = scene.safe_z
        above_place.t[2] = scene.safe_z
        move(above_pick, None, i, f"{block.name}: approach", joint=i == 0)
        grip(block.size[1] + scene.margin, None, i, f"{block.name}: pre-grasp opening")
        move(pick, None, i, f"{block.name}: descend")
        grip(block.size[1], None, i, f"{block.name}: close gripper")
        move(above_pick, i, i, f"{block.name}: lift")
        action = "rotate upright and transfer" if block.side_approach else "transfer"
        move(
            above_place,
            i,
            i,
            f"{block.name}: {action}",
            joint=bool(block.side_approach),
        )
        if block.side_approach:
            move(approach, i, i, f"{block.name}: descend outside tower")
        settle = place * SE3.Tz(-0.012)
        move(settle, i, i, f"{block.name}: align")
        move(place, i, i, f"{block.name}: precision placement", precise=True)
        grip(block.size[1] + scene.margin, None, i + 1, f"{block.name}: release")
        if block.side_approach:
            move(approach, None, i + 1, f"{block.name}: withdraw sideways")
        move(above_place, None, i + 1, f"{block.name}: retreat")
    append(
        rtb.jtraj(frames[-1].q, robot.qr, 60).q,
        3,
        scene.opening,
        None,
        8,
        "Withdraw to ready",
    )
    append(
        np.repeat(robot.qr[None, :], 2, axis=0),
        scene.observe,
        scene.opening,
        None,
        8,
        "Observe completed bridge (kinematic simulation)",
    )
    qs = np.array([f.q for f in frames])
    if np.any(qs < robot.qlim[0] - 1e-8) or np.any(qs > robot.qlim[1] + 1e-8):
        raise ValueError("Trajectory violates a joint limit")
    if frames[-1].t > 300:
        raise ValueError(
            f"Plan takes {frames[-1].t:.1f} s, exceeding the five-minute limit"
        )
    validate_payloads(robot, scene, frames)
    return frames


def validate_payloads(robot: rtb.Robot, scene: Scene, frames: list[Frame]) -> None:
    """Conservatively check carried-block bounds at samples and midpoints.

    :param robot: Kinematic model used to compute payload poses.
    :param scene: Scene with fixed banks and block poses.
    :param frames: Planned trajectory.
    :returns: None; raises if payload bounds intersect an obstacle.

    This checks the payload only, not robot links or gripper fingers.
    """
    corners = np.array(list(product((-0.5, 0.5), repeat=3))).T

    def bounds(size: np.ndarray, pose: SE3) -> tuple[np.ndarray, np.ndarray]:
        points = pose * (corners * size[:, None])
        return points.min(axis=1), points.max(axis=1)

    fixed = [bounds(size, pose) for size, pose in scene.banks]
    sources = [bounds(b.size, b.source) for b in scene.blocks]
    targets = [bounds(b.size, b.target) for b in scene.blocks]
    attachments = [grasp_pose(b).inv() * b.source for b in scene.blocks]
    for index, frame in enumerate(frames):
        if frame.held is None:
            continue
        next_frame = frames[min(index + 1, len(frames) - 1)]
        for q in (frame.q, (frame.q + next_frame.q) / 2):
            block = scene.blocks[frame.held]
            low, high = bounds(block.size, robot.fkine(q) * attachments[frame.held])
            if low[2] < -0.0002:
                raise ValueError(f"Payload intersects work surface at {frame.stage}")
            obstacles = fixed + [
                targets[i] if i < frame.placed else sources[i]
                for i in range(8)
                if i != frame.held
            ]
            for other_low, other_high in obstacles:
                overlap = np.minimum(high, other_high) - np.maximum(low, other_low)
                if np.all(overlap > 0.0002):
                    raise ValueError(
                        f"Payload bounds intersect an obstacle at {frame.stage}"
                    )


def sample(frames: list[Frame], times: np.ndarray, t: float) -> Frame:
    """Interpolate continuous state while preserving attachment events.

    :param frames: Precomputed plan.
    :param times: Frame timestamps.
    :param t: Desired time in seconds.
    :returns: Interpolated frame.
    """
    index = int(
        np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(frames) - 1)
    )
    a = frames[index]
    b = frames[min(index + 1, len(frames) - 1)]
    alpha = np.clip((t - a.t) / (b.t - a.t), 0, 1) if b.t > a.t else 0
    return Frame(
        t,
        a.q + alpha * (b.q - a.q),
        a.opening + alpha * (b.opening - a.opening),
        a.held,
        a.placed,
        a.stage,
    )


def run_viewer(
    robot: rtb.Robot, scene: Scene, frames: list[Frame], headless: bool = False
) -> None:
    """Display a replayable Swift scene with a single timeline.

    :param robot: Panda model.
    :param scene: Validated scene.
    :param frames: Planned motion.
    :param headless: Exercise the complete replay without a browser.
    :returns: None.
    """
    from spatialgeometry import Cuboid
    from swift import Button, Label, Slider, Swift

    env = Swift()
    env.launch(headless=headless, ground_opacity=0.2)
    handle = env.add_robot(robot)
    handle.control_mode = "p"
    shapes = []
    for block in scene.blocks:
        shape = Cuboid(block.size, pose=block.source, color=block.color)
        env.add(shape)
        shapes.append(shape)
    for size, pose in scene.banks:
        env.add(Cuboid(size, pose=pose, color=(0.65, 0.70, 0.76, 1)))
    # A thin work surface: its top is the z=0 reference plane.
    env.add(
        Cuboid(
            [0.95, 1.25, 0.02], pose=SE3(0.4, -0.08, -0.01), color=(0.80, 0.83, 0.87, 1)
        )
    )
    env.add(Label("Assignment 4 | Robotic Bridge Construction"))
    if scene.blocks[0].side_approach:
        env.add(
            Label(
                "4 upright columns + 2 tower caps + 2 stacked deck layers", compact=True
            )
        )
        env.add(
            Label(
                "Columns: rotate, approach from front, place slowly, withdraw sideways",
                compact=True,
            )
        )
    if scene.example:
        env.add(
            Label(
                "EXAMPLE dimensions: replace scene.json with lab measurements",
                compact=True,
            )
        )
        env.add(
            Label(
                "Assumed small-block unit: cm; deck thickness: 18 mm PLACEHOLDER",
                compact=True,
            )
        )
    env.add(
        Label(
            "Kinematic replay; contact, friction and stability are not simulated",
            compact=True,
        )
    )
    status, readout = Label("", compact=True), Label("", compact=True)
    env.add(status)
    env.add(readout)
    state = {"time": 0.0, "playing": False, "speed": 1.0, "last": None}
    times = np.array([f.t for f in frames])
    duration = float(times[-1])

    def play(_: object) -> None:
        if state["time"] >= duration:
            state["time"] = 0.0
        state["playing"] = not state["playing"]

    def reset(_: object) -> None:
        state.update(time=0.0, playing=False)

    def seek(value: float) -> None:
        state.update(time=float(value), playing=False)

    env.add(Button(play, label="Play / Pause"))
    env.add(Button(reset, label="Reset"))
    env.add(
        Slider(
            lambda v: state.update(speed=float(v)),
            min=0.25,
            max=3,
            step=0.25,
            value=1,
            label="Playback speed",
            unit="x",
        )
    )
    timeline = Slider(
        seek,
        min=0,
        max=duration,
        step=0.1,
        value=0,
        label="Seek to time (pauses playback)",
        unit="s",
        precision=1,
    )
    env.add(timeline)
    attachments = [grasp_pose(b).inv() * b.source for b in scene.blocks]

    def update(clock: float, _: dict) -> np.ndarray:
        last = state["last"]
        dt = 0.0 if last is None else max(0, clock - last)
        state["last"] = clock
        if state["playing"]:
            state["time"] = min(duration, state["time"] + dt * state["speed"])
            if state["time"] >= duration:
                state["playing"] = False
        frame = sample(frames, times, state["time"])
        robot.grippers[0].q = np.full(2, frame.opening / 2)
        tcp = robot.fkine(frame.q)
        for i, (shape, block) in enumerate(zip(shapes, scene.blocks)):
            shape.T = (
                tcp * attachments[i]
                if frame.held == i
                else block.target
                if i < frame.placed
                else block.source
            )
        status.label = f"{frame.placed}/8 placed | {frame.t:.1f} / {duration:.1f} s<br>{html.escape(frame.stage)}"
        readout.label = "TCP [m]: " + ", ".join(f"{v:.3f}" for v in tcp.t)
        # Swift's frontend reports programmatic Slider updates as input events.
        # Keep this slider input-only; the status label shows live elapsed time.
        return frame.q

    handle.callback = update
    if hasattr(env, "set_camera_pose") and not headless:
        env.set_camera_pose([1.2, -1.3, 1.0], [0.43, -0.03, 0.1])
    try:
        if headless:
            for t in np.arange(0, duration + 0.1, 0.1):
                state["time"] = min(float(t), duration)
                env.step(0.1)
            for shape, block in zip(shapes, scene.blocks):
                if not np.allclose(shape.T, block.target.A, atol=1e-6):
                    raise ValueError(f"Final block pose mismatch: {block.name}")
            print("Headless Swift replay passed: 8/8 blocks in their target poses")
        else:
            env.run(dt=1 / 30)
    finally:
        env.close()


def main() -> None:
    """Load, plan, validate and optionally display the bridge rehearsal.

    :returns: None.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).with_name("scene.json")
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate and plan without opening a browser",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="also run the full Swift replay without a browser",
    )
    args = parser.parse_args()
    scene = load_scene(args.config)
    robot = rtb.models.Panda()
    frames = build_plan(robot, scene)
    print(
        f"Plan valid: {len(frames)} samples, {frames[-1].t:.1f} s including observation",
        flush=True,
    )
    if not args.check or args.headless:
        run_viewer(robot, scene, frames, headless=args.headless)


if __name__ == "__main__":
    main()
