import logging
import time

import dimos.core as core
from dimos.vr.modules import MetaQuestModule

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_metaquest_module():
    dimos = core.start(1)
    
    try:
        quest = dimos.deploy(
            MetaQuestModule,
            host="0.0.0.0",
            port=8881
        )
        
        logger.info("Generating SSL certificate...")
        cert_result = quest.generate_certificate().result()
        logger.info(f"Certificate result: {cert_result}")
        
        logger.info("Starting VR server...")
        quest.start().result()
        
        stats = quest.get_stats().result()
        logger.info(f"Server stats: {stats}")
        
        logger.info("VR server running. Access at: https://localhost:8881")
        logger.info("Connect with Quest 3 to test controller streaming")
        
        time.sleep(30)
        
        final_stats = quest.get_stats().result()
        logger.info(f"Final stats: {final_stats}")
        
        quest.stop().result()
        logger.info("VR server stopped")
        
    finally:
        dimos.shutdown()


if __name__ == "__main__":
    test_metaquest_module()
