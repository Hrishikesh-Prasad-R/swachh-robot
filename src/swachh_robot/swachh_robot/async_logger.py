#!/usr/bin/env python3
"""
Async File Logger — Non-blocking dual-sink logging.

Two log files:
  logs/full.log      — DEBUG, INFO, WARN, ERROR
  logs/essential.log  — INFO, WARN, ERROR only

Usage:
  from swachh_robot.async_logger import get_logger
  logger = get_logger('node_name')
  logger.debug('low-level detail')
  logger.info('normal event')
  logger.warning('something unusual')
  logger.error('failure')
"""

import logging
import logging.handlers
import os
import sys
import queue
from datetime import datetime

# Log directory
LOG_DIR = os.path.join(
    os.path.expanduser('~'),
    'Desktop', 'swachh-robot-simulation', 'logs'
)

# Module-level state
_listener = None
_log_queue = None
_initialized = False


def _ensure_log_dir():
    """Create log directory if it doesn't exist."""
    os.makedirs(LOG_DIR, exist_ok=True)


def _init_queue_listener():
    """Initialize the shared QueueListener with file + terminal handlers."""
    global _listener, _log_queue, _initialized

    if _initialized:
        return _log_queue

    _ensure_log_dir()
    _log_queue = queue.Queue(-1)  # Unbounded

    # Timestamp format
    fmt = logging.Formatter(
        '%(asctime)s.%(msecs)03d [%(levelname)-5s] [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Full log: DEBUG and above
    full_handler = logging.FileHandler(
        os.path.join(LOG_DIR, 'full.log'), mode='a'
    )
    full_handler.setLevel(logging.DEBUG)
    full_handler.setFormatter(fmt)

    # Essential log: INFO and above
    essential_handler = logging.FileHandler(
        os.path.join(LOG_DIR, 'essential.log'), mode='a'
    )
    essential_handler.setLevel(logging.INFO)
    essential_handler.setFormatter(fmt)

    # Terminal handler: INFO+ live in terminal (raw-mode safe \r\n)
    terminal_handler = _RawTerminalHandler()
    terminal_handler.setLevel(logging.INFO)
    terminal_handler.setFormatter(logging.Formatter(
        '[%(levelname)-5s] [%(name)s] %(message)s'
    ))

    # QueueListener processes handlers on a background thread
    _listener = logging.handlers.QueueListener(
        _log_queue, full_handler, essential_handler, terminal_handler,
        respect_handler_level=True
    )
    _listener.start()
    _initialized = True

    return _log_queue


class _RawTerminalHandler(logging.StreamHandler):
    """StreamHandler that uses \\r\\n for raw terminal mode compatibility."""

    def __init__(self):
        super().__init__(sys.stderr)

    def emit(self, record):
        try:
            msg = self.format(record)
            # \r\n needed because terminal is in raw mode (no auto CR)
            self.stream.write('\r\n' + msg)
            self.stream.flush()
        except Exception:
            self.handleError(record)


def get_logger(name: str) -> logging.Logger:
    """
    Get a non-blocking async logger.

    All log calls are queued and written to files by a background thread.
    No terminal output. No blocking I/O in the calling thread.
    """
    log_queue = _init_queue_listener()

    logger = logging.getLogger(f'swachh.{name}')
    logger.setLevel(logging.DEBUG)

    # Prevent duplicate handlers on repeated calls
    if not logger.handlers:
        qh = logging.handlers.QueueHandler(log_queue)
        logger.addHandler(qh)

    # Prevent propagation to root logger (no terminal output)
    logger.propagate = False

    return logger


def shutdown_logger():
    """Flush and stop the background listener. Call on node shutdown."""
    global _listener, _initialized
    if _listener:
        _listener.stop()
        _listener = None
    _initialized = False
