"""Model-independent rollout storage with camera videos, NumPy, and JSON.

Episodes contain pre-action observations and matching actions. Embeddings are
sampled at policy-query frequency rather than the environment step rate.
No training-framework dependency is imported by this module.
"""

import json
import shutil
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator

import imageio.v2 as imageio
import numpy as np


STEP_FIELDS = ("image", "wrist_image", "state", "action")
ARRAY_FIELDS = ("state", "action", "embedding")
CAMERA_VIDEOS = {"image": "overview.mp4", "wrist_image": "wrist.mp4"}


@dataclass
class Trajectory:
    """One rollout using the conventions in docs/dataset.md.

    Args:
        prompt: Task language instruction.
        success: Whether LIBERO's task success predicate was reached.
        image: T agent-view RGB uint8 frames, each shaped (H, W, 3).
        wrist_image: T wrist-camera RGB uint8 frames with the same convention.
        state: T vectors of EEF position, axis-angle orientation, gripper qpos.
        action: T seven-dimensional simulator OSC_POSE controller commands.
        embedding: Q policy embeddings in inference-query order.
        metadata: JSON-compatible environment, trial, and policy provenance.
    """

    prompt: str | None = None
    success: bool = False
    image: list = field(default_factory=list)
    wrist_image: list = field(default_factory=list)
    state: list = field(default_factory=list)
    action: list = field(default_factory=list)
    embedding: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.action)


class TrajectoryDataset:
    """Append-only episode directory readable without a VLA SDK.

    Args:
        dataset_dir: Dataset root, created if missing. Existing episodes are
            reused. Episodes are committed through atomic directory renames.
            A single writer must own a dataset at a time.
    """

    def __init__(self, dataset_dir: str | Path):
        self.dataset_dir = Path(dataset_dir)
        self.dataset_dir.mkdir(parents=True, exist_ok=True)

    def __len__(self):
        return len(list(self.dataset_dir.glob("ep_[0-9][0-9][0-9][0-9][0-9]")))

    def __iter__(self) -> Iterator[Trajectory]:
        for index in range(len(self)):
            yield self[index]

    def __getitem__(self, index: int) -> Trajectory:
        if index < 0 or index >= len(self):
            raise IndexError(index)
        directory = self.dataset_dir / f"ep_{index:05d}"
        metadata = json.loads((directory / "metadata.json").read_text())
        version = metadata.pop("schema_version")
        if version not in (1, 2):
            raise ValueError("Unsupported trajectory schema version")
        num_steps = metadata.pop("num_steps")
        with np.load(directory / "arrays.npz", allow_pickle=False) as arrays:
            fields = (*CAMERA_VIDEOS, *ARRAY_FIELDS) if version == 1 else ARRAY_FIELDS
            values = {name: arrays[name] for name in fields}
        if version == 2:
            for name, filename in CAMERA_VIDEOS.items():
                with imageio.get_reader(directory / filename, format="mp4") as reader:
                    values[name] = np.stack(list(reader.iter_data()))
        if any(len(values[name]) != num_steps for name in STEP_FIELDS):
            raise ValueError("Episode images, states, and actions must match num_steps")
        return Trajectory(**metadata, **values)

    def write_episode(self, trajectory: Trajectory, fps: int | None = None) -> int:
        """Append a synchronized rollout and return its zero-based episode ID.

        All step arrays must have T entries; embeddings have Q entries in
        inference-query order. fps records the control frequency.
        Numeric arrays are compressed without pickled objects. Overview and wrist
        frames are stored in separate H.264 videos using ImageIO's default
        compression settings. Frame ordering and counts match the numeric steps;
        decoded pixels are lossy and non-aligned image dimensions may be resized.

        Args:
            trajectory: Nonempty rollout with T synchronized step arrays and Q embeddings.
            fps: Optional control frequency in Hz; defaults to episode metadata or 20.

        Returns:
            Zero-based integer ID of the newly committed episode.
        """
        fps = fps if fps is not None else trajectory.metadata.get("control_frequency_hz", 20)
        if fps <= 0:
            raise ValueError("Control frequency must be positive")
        if not trajectory.prompt or len(trajectory) == 0:
            raise ValueError("Cannot write a trajectory without a prompt and actions")
        arrays = {name: np.asarray(getattr(trajectory, name)) for name in (*STEP_FIELDS, "embedding")}
        if any(len(arrays[name]) != len(trajectory) for name in STEP_FIELDS):
            raise ValueError("Images, states, and actions must have equal lengths")
        if any(array.dtype == object for array in arrays.values()):
            raise ValueError("Episode arrays must have consistent numeric shapes")
        for name in ("image", "wrist_image"):
            if arrays[name].dtype != np.uint8 or arrays[name].ndim != 4 or arrays[name].shape[-1] != 3:
                raise ValueError(f"{name} must be uint8 RGB frames of shape (T, H, W, 3)")
        if arrays["action"].shape != (len(trajectory), 7):
            raise ValueError("Actions must have shape (T, 7)")
        index = len(self)
        directory = self.dataset_dir / f"ep_{index:05d}"
        temporary = Path(tempfile.mkdtemp(prefix=".episode-", dir=self.dataset_dir))
        metadata = {
            "schema_version": 2,
            "num_steps": len(trajectory),
            "prompt": trajectory.prompt,
            "success": bool(trajectory.success),
            "metadata": {**trajectory.metadata, "control_frequency_hz": fps},
        }
        try:
            np.savez_compressed(temporary / "arrays.npz", **{name: arrays[name] for name in ARRAY_FIELDS})
            (temporary / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
            for name, filename in CAMERA_VIDEOS.items():
                with imageio.get_writer(temporary / filename, fps=fps, codec="libx264") as writer:
                    for frame in arrays[name]:
                        writer.append_data(frame)
            temporary.rename(directory)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
        return index

    def write_manifest(self) -> Path:
        """Write manifest.jsonl for completed episodes; return its path.

        Returns:
            Path to the atomically replaced manifest.jsonl file.
        """
        path = self.dataset_dir / "manifest.jsonl"
        temporary = path.with_suffix(".jsonl.tmp")
        with temporary.open("w") as stream:
            for index in range(len(self)):
                directory = self.dataset_dir / f"ep_{index:05d}"
                metadata = json.loads((directory / "metadata.json").read_text())
                stream.write(json.dumps({"episode_id": index, "path": directory.name, **metadata}) + "\n")
        temporary.replace(path)
        return path


def embedding_collate(batch: Iterable[Trajectory]):
    """Collect the first available policy embedding from each rollout.

    Args:
        batch: Trajectories with policy embeddings of a common dimension D.

    Returns:
        Float32 torch tensor of shape (N, D), omitting rollouts without embeddings.

    Raises:
        ValueError: No rollout contains an embedding.
    """
    import torch

    embeddings = [np.asarray(traj.embedding[0]) for traj in batch if len(traj.embedding)]
    if not embeddings:
        raise ValueError("No policy embeddings collected; check the server and collection configuration")
    return torch.as_tensor(np.stack(embeddings), dtype=torch.float32)
