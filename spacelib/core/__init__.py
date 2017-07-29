import logging

import krpc

logger = logging.getLogger(__name__)

def setup():
    logger.info("Connecting to kRPC...")
    conn = krpc.connect(name="spacelib")
    logger.info("Connected.")
    return conn
