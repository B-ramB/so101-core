"""The interfaces every so101 follower and leader has, real or simulated. Joint values in model radians.

Code that controls an arm (the teleop loop, a controller, the safety wrapper) is written against `Follower` /
`Leader`, so it runs unchanged on the real arm and in the sim:

    Follower                  Leader
    ├── RealFollower          └── RealLeader       real arms via lerobot (so101.hardware.adapters), deg <-> rad
    ├── MujocoFollower                             the MuJoCo sim (so101.sim.follower), radians natively
    └── SafeFollower                               wraps either: validates, clamps, limits the step (so101.safety)

These are Protocols: the classes don't inherit from them, they just have these members. Keys follow lerobot:
`"<joint>.pos"` for joint positions (radians), the camera name (`"wrist"`) for an HxWx3 image.
"""

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class Follower(Protocol):
    @property
    def observation_features(self) -> dict[str, Any]: ...

    @property
    def action_features(self) -> dict[str, Any]: ...

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def get_observation(self) -> dict[str, Any]:
        """Joint positions in radians (`"<joint>.pos"`), plus camera images."""
        ...

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        """Set joint targets in radians (a partial action only moves the given joints); returns what was sent."""
        ...


@runtime_checkable
class Leader(Protocol):
    @property
    def action_features(self) -> dict[str, Any]: ...

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def get_action(self) -> dict[str, float]:
        """The leader's joint positions in radians (`"<joint>.pos"`)."""
        ...
