import logging

import krpc

logger = logging.getLogger(__name__)

# TODO telemetry streaming display

conn = None
def setup():
    global conn
    logger.info("Connecting to kRPC...")
    conn = krpc.connect(name="spacelib")
    logger.info("Connected.")
    return conn

def init_logging():
    logger = logging.getLogger()
    if not logger.handlers:
        logger.setLevel(logging.DEBUG)
        handler = logging.StreamHandler()
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(asctime)s|%(name)s|%(levelname)s\t%(message)s")
        handler.setFormatter(formatter)
        logger.addHandler(handler)
