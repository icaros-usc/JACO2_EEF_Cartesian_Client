"""Optional pre-action rollout recording in the shared trajectory format."""

import time
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryFile

import numpy as np
from imageio_ffmpeg import get_ffmpeg_exe

from dataset_utils import Trajectory, TrajectoryDataset

from .transport import image_rgb


class RolloutRecorder:
    """Buffer one launch's accepted actions and save an episode after stopping.

    Images are unscaled RGB uint8. States are world-frame XYZ (metres), rotation
    vectors (radians), and two proximal finger angles (radians). Actions retain
    the policy's seven-dimensional world-frame deltas and normalized gripper.
    """

    def __init__(self, directory, instruction, config):
        get_ffmpeg_exe()
        self.dataset = TrajectoryDataset(Path(directory).expanduser())
        with TemporaryFile(dir=self.dataset.dataset_dir):
            pass
        self.started_at = time.monotonic()
        self.saved = False
        self.trajectory = Trajectory(
            prompt=instruction,
            metadata={
                "source": "jaco_vla",
                "started_at": datetime.now(timezone.utc).isoformat(),
                "success_known": False,
                "state_frame": "world",
                "action_frame": "world",
                "action_units": "policy",
                "client_config": {
                    key: value.tolist() if isinstance(value, np.ndarray) else value
                    for key, value in config.items()
                },
                "step_timestamps_s": [],
                "embedding_step_indices": [],
            },
        )

    def append(self, overview, wrist, state, action, observed_at):
        """Append an accepted action and its dispatch-time observation snapshot.

        ROS image messages must be immutable snapshots. State has shape (8,),
        action has shape (7,), and observed_at is monotonic time in seconds.
        """
        # Decode both frames before changing the buffer so a bad frame adds no step.
        images = [np.asarray(image_rgb(msg)).copy() for msg in (overview, wrist)]
        for name, image in zip(("image", "wrist_image"), images, strict=True):
            frames = getattr(self.trajectory, name)
            if frames and image.shape != frames[0].shape:
                raise ValueError(f"{name} resolution changed during the rollout")
        self.trajectory.image.append(images[0])
        self.trajectory.wrist_image.append(images[1])
        self.trajectory.state.append(np.array(state, dtype=np.float32, copy=True))
        self.trajectory.action.append(np.array(action, copy=True))
        self.trajectory.metadata["step_timestamps_s"].append(
            observed_at - self.started_at
        )

    def append_embedding(self, embedding):
        """Store an optional server embedding once per query, before its actions."""
        if embedding is not None:
            embedding = np.array(embedding, copy=True)
            if embedding.dtype.kind not in "buif" or not np.isfinite(embedding).all():
                raise ValueError("Policy embedding must contain finite numbers")
            embeddings = self.trajectory.embedding
            if embeddings and embedding.shape != embeddings[0].shape:
                raise ValueError("Policy embedding shape changed during the rollout")
            embeddings.append(embedding)
            self.trajectory.metadata["embedding_step_indices"].append(
                len(self.trajectory)
            )

    def save(self, reason):
        """Commit a nonempty episode once; return its directory or None.

        Video uses the average observed action rate; step_timestamps_s preserves
        irregular timing, including policy latency. Task success is unknown.
        """
        if self.saved or not self.trajectory:
            return None
        self.trajectory.metadata.update(
            ended_at=datetime.now(timezone.utc).isoformat(), stop_reason=reason
        )
        timestamps = self.trajectory.metadata["step_timestamps_s"]
        fps = (
            (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
            if len(timestamps) > 1 and timestamps[-1] > timestamps[0]
            else 20
        )
        index = self.dataset.write_episode(self.trajectory, fps=fps)
        self.saved = True
        self.dataset.write_manifest()
        return self.dataset.dataset_dir / f"ep_{index:05d}"
