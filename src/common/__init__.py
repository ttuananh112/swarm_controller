import logging
import sys

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger_handler = logging.StreamHandler(stream=sys.stdout)
logger_handler.setFormatter(logging.Formatter(
    '%(asctime)s [%(levelname)-5s] %(module)-10s %(funcName)-10s %(message)s')
)
logger.addHandler(logger_handler)
