import logging
import logging.config
from datetime import datetime

from .config import ROS_LOG_DIR

ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
ROS_INFO_FILE = ROS_LOG_DIR / f"ros_info_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

ROS_INFO_LOGGER_NAME = "ros_info_only"
logging.config.dictConfig(
    {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {"std": {"format": "%(asctime)s %(message)s"}},
        "handlers": {
            "ros_info_file": {
                "class": "logging.handlers.RotatingFileHandler",
                "filename": str(ROS_INFO_FILE),
                "maxBytes": 5 * 1024 * 1024,
                "backupCount": 5,
                "encoding": "utf-8",
                "formatter": "std",
            }
        },
        "loggers": {
            ROS_INFO_LOGGER_NAME: {
                "handlers": ["ros_info_file"],
                "level": "INFO",
                "propagate": False,
            }
        },
    }
)

ros_info_logger = logging.getLogger(ROS_INFO_LOGGER_NAME)
