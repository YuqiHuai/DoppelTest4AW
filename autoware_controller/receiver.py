import atexit
import signal
from typing import Optional

import uvicorn

from receiver_app import app
from receiver_app.config import UVICORN_HOST, UVICORN_PORT
from receiver_app.processes import (
    _stop_autoware_process,
    _stop_rosbag_process,
    _stop_sender_process,
)


_shutdown_started = False


def _shutdown(reason: Optional[str]) -> None:
    global _shutdown_started
    if _shutdown_started:
        return
    _shutdown_started = True
    _stop_rosbag_process(reason or "shutdown")
    _stop_autoware_process(reason or "shutdown")
    _stop_sender_process(reason or "shutdown")


def _handle_signal(signum: int, _frame) -> None:
    name = signal.Signals(signum).name
    _shutdown(f"signal:{name}")


if __name__ == "__main__":
    print(f"Starting Perception Server on {UVICORN_HOST}:{UVICORN_PORT}")
    atexit.register(_shutdown, "exit")
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        uvicorn.run(
            app,
            host=UVICORN_HOST,
            port=UVICORN_PORT,
            log_config=None,
            access_log=False,
        )
    finally:
        _shutdown("uvicorn_exit")
