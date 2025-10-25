"""
Progress tracking for launch2plan conversion.

Provides real-time feedback during slow operations like introspection.
"""

import sys
from dataclasses import dataclass
from typing import Optional


@dataclass
class ProgressTracker:
    """
    Simple progress tracker for conversion operations.

    Displays progress messages without fancy formatting - just clear text output.
    """

    def __init__(self, enabled: bool = True, file=None):
        """
        Initialize progress tracker.

        Args:
            enabled: If False, suppresses all output (useful for testing)
            file: Output file (defaults to sys.stdout)
        """
        self.enabled = enabled
        self.file = file if file is not None else sys.stdout
        self._current_phase: Optional[str] = None
        self._phase_total: int = 0
        self._phase_current: int = 0

    def start_phase(self, phase_name: str, total: int = 0):
        """
        Start a new phase with optional total count.

        Args:
            phase_name: Name of the phase (e.g., "Introspecting nodes")
            total: Total items to process (0 if unknown)
        """
        if not self.enabled:
            return

        self._current_phase = phase_name
        self._phase_total = total
        self._phase_current = 0

        if total > 0:
            self._print(f"{phase_name}... (0/{total})")
        else:
            self._print(f"{phase_name}...")

    def update(self, item_name: Optional[str] = None):
        """
        Update progress for current phase.

        Args:
            item_name: Name of current item being processed (optional)
        """
        if not self.enabled or not self._current_phase:
            return

        self._phase_current += 1

        if self._phase_total > 0:
            # Show counter with item name
            prefix = f"  [{self._phase_current}/{self._phase_total}]"
            if item_name:
                self._print(f"{prefix} {item_name}")
            else:
                # Inline update - use carriage return
                self._print(
                    f"\r{self._current_phase}... ({self._phase_current}/{self._phase_total})",
                    end="",
                )
        else:
            # No total count - just show item being processed
            if item_name:
                self._print(f"  - {item_name}")

    def end_phase(self, message: Optional[str] = None):
        """
        End current phase with optional completion message.

        Args:
            message: Completion message (e.g., "✓ Discovered 46 files")
        """
        if not self.enabled or not self._current_phase:
            return

        # Clear any inline progress
        if self._phase_total > 0 and self._phase_current > 0:
            self._print()  # New line after inline updates

        if message:
            self._print(message)

        self._current_phase = None
        self._phase_total = 0
        self._phase_current = 0

    def message(self, text: str):
        """
        Print a standalone message (not part of current phase).

        Args:
            text: Message to print
        """
        if not self.enabled:
            return
        self._print(text)

    def _print(self, text: str = "", end: str = "\n"):
        """Internal print helper."""
        print(text, end=end, file=self.file, flush=True)


# Global progress tracker instance (can be replaced for testing)
_global_tracker: Optional[ProgressTracker] = None


def get_progress_tracker() -> ProgressTracker:
    """Get or create the global progress tracker."""
    global _global_tracker
    if _global_tracker is None:
        _global_tracker = ProgressTracker()
    return _global_tracker


def set_progress_tracker(tracker: ProgressTracker):
    """Set the global progress tracker (useful for testing)."""
    global _global_tracker
    _global_tracker = tracker
