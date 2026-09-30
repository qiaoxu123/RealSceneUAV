from __future__ import annotations

from abc import ABC, abstractmethod

from realsceneuav.core.types import ControlCommand


class Controller(ABC):
    @abstractmethod
    def poll(self) -> ControlCommand:
        raise NotImplementedError
