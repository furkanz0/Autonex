"""
utils/logger.py — Console logging helpers
"""

import sys
import io

# Fix Windows cp1254 console not being able to print UTF-8 characters
if sys.stdout.encoding and sys.stdout.encoding.lower().startswith("cp"):
    try:
        sys.stdout = io.TextIOWrapper(
            sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True
        )
    except Exception:
        pass


def log(m, s="✓"):
    """Single line status log."""
    print(f"  [{s}] {m}")


def sec(t):
    """Print section header."""
    print(f"\n{'═'*62}\n  {t}\n{'═'*62}")
