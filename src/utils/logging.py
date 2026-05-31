"""Structured logging setup for the GWM-UAV framework.

Usage::

    from src.utils.logging import setup_logging

    logger = setup_logging(level="DEBUG", log_file="run.log")
    logger.info("Training started")
"""

import logging
import sys
from typing import Optional

_LOG_FORMAT = "[%(asctime)s] %(levelname)s %(name)s: %(message)s"


def setup_logging(
    level: str = "INFO",
    log_file: Optional[str] = None,
) -> logging.Logger:
    """Configure and return the framework-wide logger.

    Creates (or retrieves) a logger named ``gwm_uav`` and attaches:

    * A **console** handler that writes to *stderr*.
    * An optional **file** handler when *log_file* is provided.

    Calling this function multiple times is safe — duplicate handlers are
    avoided by clearing existing handlers before reconfiguring.

    Args:
        level: Logging level string (e.g. ``"DEBUG"``, ``"INFO"``).
        log_file: Optional path to a log file.  If ``None``, only the
            console handler is attached.

    Returns:
        The configured :class:`logging.Logger` instance.
    """
    logger = logging.getLogger("gwm_uav")
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))

    # Prevent duplicate handlers on repeated calls
    logger.handlers.clear()

    formatter = logging.Formatter(_LOG_FORMAT)

    # Console handler
    console_handler = logging.StreamHandler(sys.stderr)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # Optional file handler
    if log_file is not None:
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger
