from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from realsceneuav.core.types import ControlCommand


class Controller(ABC):
    @abstractmethod
    def poll(self) -> ControlCommand:
        raise NotImplementedError

    def provenance(self) -> dict[str, Any]:
        """Serializable description of the input backend used for an episode."""

        return {"backend": type(self).__name__}
