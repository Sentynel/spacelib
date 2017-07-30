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
