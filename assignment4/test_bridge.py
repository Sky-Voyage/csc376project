"""Geometry, motion and replay regression checks for the bridge rehearsal."""

import json
from pathlib import Path

import numpy as np
import pytest
from bridge import build_plan, grasp_pose, load_scene, run_viewer, sample

import roboticstoolbox as rtb

CONFIG = Path(__file__).with_name("scene.json")


def test_measured_layout_and_bearing() -> None:
    scene = load_scene(CONFIG)
    assert len(scene.blocks) == 8
    assert all(np.isclose(pose.t[2] + size[2] / 2, 0.125) for size, pose in scene.banks)
    for post in scene.blocks[:4]:
        assert np.allclose(post.target.R[:, 0], [0, 0, 1], atol=1e-12)
        assert np.isclose(post.target.t[2] - post.size[0] / 2, 0.125)
        assert post.side_approach == 0.080
        assert np.isclose(grasp_pose(post, True).t[2], 0.182)
    for cap in scene.blocks[4:6]:
        assert np.isclose(cap.target.t[2] - cap.size[2] / 2, 0.199)
        assert np.allclose(cap.target.R[:, 0], [0, 1, 0], atol=1e-12)
    assert np.isclose(scene.blocks[6].target.t[2] - scene.blocks[6].size[2] / 2, 0.212)
    assert np.allclose(scene.blocks[6].target.t[:2], scene.blocks[7].target.t[:2])
    assert np.isclose(
        scene.blocks[6].target.t[2] + scene.blocks[6].size[2] / 2,
        scene.blocks[7].target.t[2] - scene.blocks[7].size[2] / 2,
    )
    # Lower deck overlaps each tower cap by 20 mm in the span direction.
    deck = scene.blocks[6]
    for cap in scene.blocks[4:6]:
        overlap = min(
            deck.target.t[0] + deck.size[0] / 2, cap.target.t[0] + cap.size[1] / 2
        ) - max(deck.target.t[0] - deck.size[0] / 2, cap.target.t[0] - cap.size[1] / 2)
        assert np.isclose(overlap, 0.020)


def test_flat_layout_remains_available() -> None:
    scene = load_scene(CONFIG.with_name("scene.flat.json"))
    assert all(
        np.isclose(b.target.t[2] - b.size[2] / 2, 0.138) for b in scene.blocks[6:]
    )


@pytest.mark.parametrize(
    "change",
    [
        {"deck_block_mm": [100, 90, 18]},
        {"bank_inner_gap_mm": 110},
        {"tower_column_spacing_mm": 20},
        {"tower_grasp_offset_mm": 34},
        {"tower_standoff_mm": 20},
        {"safe_tcp_z_mm": 150},
        {"layout": "cross_stack"},
        {"observe_seconds": 5},
        {"pier_block_mm": [74, 23, None]},
    ],
)
def test_reject_invalid_scene(tmp_path: Path, change: dict) -> None:
    config = json.loads(CONFIG.read_text())
    config.update(change)
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_scene(path)


@pytest.fixture(scope="module")
def planned_scene() -> tuple:
    scene = load_scene(CONFIG)
    robot = rtb.models.Panda()
    frames = build_plan(robot, scene)
    return robot, scene, frames


def test_full_plan_limits_attachment_and_release(planned_scene: tuple) -> None:
    robot, scene, frames = planned_scene
    times = np.array([f.t for f in frames])
    qs = np.array([f.q for f in frames])
    assert frames[-1].placed == 8
    assert times[-1] < 300
    assert np.all(qs >= robot.qlim[0] - 1e-8)
    assert np.all(qs <= robot.qlim[1] + 1e-8)
    dt = np.diff(times)
    moving = dt > 1e-10
    speeds = np.abs(np.diff(qs, axis=0)[moving]) / dt[moving, None]
    assert np.all(speeds <= robot.qdlim[:7] * 0.35 + 1e-8)
    for i, block in enumerate(scene.blocks):
        held = [f for f in frames if f.held == i]
        attachment = grasp_pose(block).inv() * block.source
        first_pose = robot.fkine(held[0].q) * attachment
        final_pose = robot.fkine(held[-1].q) * attachment
        assert np.allclose(first_pose.A, block.source.A, atol=1e-4)
        assert np.allclose(final_pose.A, block.target.A, atol=1e-4)
    # Seeking backwards restores the correct discrete task state.
    assert sample(frames, times, times[-1]).placed == 8
    assert sample(frames, times, 0).placed == 0
    assert np.allclose(sample(frames, times, 0).q, robot.qr)


def test_play_survives_frontend_slider_echo(
    monkeypatch: pytest.MonkeyPatch, planned_scene: tuple
) -> None:
    """Exercise buttons and emulate Swift's update_element -> input echo."""
    from swift import Button, Label, Slider, Swift

    original_launch = Swift.launch
    monkeypatch.setattr(
        Swift, "launch", lambda self, **kw: original_launch(self, headless=True)
    )
    monkeypatch.setattr(Swift, "set_camera_pose", lambda *args: None)

    def emulate_browser(env: Swift, **kwargs: object) -> None:
        play = next(
            e
            for e in env.elements.values()
            if isinstance(e, Button) and e.label == "Play / Pause"
        )
        reset = next(
            e
            for e in env.elements.values()
            if isinstance(e, Button) and e.label == "Reset"
        )
        seek = next(
            e
            for e in env.elements.values()
            if isinstance(e, Slider) and e.label.startswith("Seek")
        )
        env.step(0.1)
        play.cb(0)
        for _ in range(20):
            env.step(0.1)
            # ui.js Slider.update() emits an input event on every Python update.
            for key, element in env.elements.items():
                if isinstance(element, Slider) and element._changed:
                    element._changed = False
                    env.process_events({key: element.value})
        status = next(
            e
            for e in env.elements.values()
            if isinstance(e, Label) and "/8 placed" in e.label
        )
        assert "2.0 /" in status.label, status.label
        reset.cb(0)
        env.step(0.1)
        assert "0.0 /" in status.label
        seek.cb(60.0)
        env.step(0.1)
        assert "60.0 /" in status.label
        play.cb(0)
        env.step(0.1)
        assert "60.1 /" in status.label

    monkeypatch.setattr(Swift, "run", emulate_browser)
    run_viewer(*planned_scene)
