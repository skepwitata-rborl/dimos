import asyncio
import json
import logging
import threading
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from dimos.core import Module, Out, rpc
from ..models import ControllerData, ControllerFrame

logger = logging.getLogger(__name__)


class MetaQuestModule(Module):
    """MetaQuest VR controller module for WebXR-based controller streaming."""
    
    controller_left: Out[ControllerData] = None
    controller_right: Out[ControllerData] = None
    controller_both: Out[ControllerFrame] = None

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8881,
        ssl_cert: Optional[str] = None,
        ssl_key: Optional[str] = None,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.host = host
        self.port = port
        self.ssl_cert = ssl_cert
        self.ssl_key = ssl_key
        
        self._server = None
        self._running = False
        
        self.stats = {
            "frames_received": 0,
            "errors": 0,
            "last_frame": None
        }
        
        self._setup_fastapi()

    def _setup_fastapi(self):
        """Initialize FastAPI application with VR routes."""
        self.app = FastAPI(title="VR Teleoperation Server", version="0.1.0")
        
        static_dir = Path(__file__).parent.parent / "static"
        if static_dir.exists():
            self.app.mount("/static", StaticFiles(directory=static_dir), name="static")
        else:
            logger.warning(f"Static directory not found: {static_dir}")
        
        self._setup_routes()

    def _setup_routes(self):
        """Setup FastAPI routes for VR interface."""
        
        @self.app.get("/", response_class=HTMLResponse)
        async def get_vr_interface():
            static_dir = Path(__file__).parent.parent / "static"
            index_path = static_dir / "index.html"
            if not index_path.exists():
                return HTMLResponse(
                    content="<h1>Error: VR interface not found</h1>", 
                    status_code=500
                )
            return HTMLResponse(content=index_path.read_text())

        @self.app.get("/health")
        async def health_check():
            return {
                "status": "ok",
                "frames_received": self.stats["frames_received"],
                "errors": self.stats["errors"],
                "running": self._running
            }

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            client_ip = websocket.client.host if websocket.client else "unknown"
            logger.info(f"VR client connected: {client_ip}")

            try:
                while True:
                    try:
                        message = await websocket.receive()
                        
                        if message["type"] == "websocket.disconnect":
                            break
                            
                    except WebSocketDisconnect:
                        break

                    data_text = message.get("text")
                    if not data_text:
                        continue

                    try:
                        frame_dict = json.loads(data_text)
                        frame = ControllerFrame(**frame_dict)
                        
                        self.stats["frames_received"] += 1
                        self.stats["last_frame"] = frame
                        
                        await self._publish_controller_data(frame)
                        
                    except ValidationError as e:
                        self.stats["errors"] += 1
                        logger.error(f"Controller data validation error: {e}")
                    except Exception as e:
                        self.stats["errors"] += 1
                        logger.error(f"Error processing controller frame: {e}")

            except WebSocketDisconnect:
                logger.info(f"VR client disconnected: {client_ip}")
            except Exception as e:
                logger.error(f"WebSocket error: {e}", exc_info=True)

    async def _publish_controller_data(self, frame: ControllerFrame):
        """Publish controller data to output streams."""
        try:
            if self.controller_both:
                self.controller_both.publish(frame)
            
            if frame.left and self.controller_left:
                self.controller_left.publish(frame.left)
            
            if frame.right and self.controller_right:
                self.controller_right.publish(frame.right)
                
        except Exception as e:
            logger.error(f"Error publishing controller data: {e}")

    def _get_ssl_config(self):
        cert_path = self.ssl_cert or "dimos/vr/certificates/cert.pem"
        key_path = self.ssl_key or "dimos/vr/certificates/key.pem"
        
        cert_file = Path(cert_path)
        key_file = Path(key_path)
        
        if cert_file.exists() and key_file.exists():
            logger.info(f"SSL enabled with cert: {cert_file}")
            return {
                "ssl_certfile": str(cert_file),
                "ssl_keyfile": str(key_file)
            }
        else:
            logger.warning("SSL certificates not found, running without HTTPS")
            logger.warning("WebXR requires HTTPS for controller access")
            return {}

    @rpc
    def start(self):
        """Start the VR teleoperation server."""
        if self._running:
            logger.warning("VR server already running")
            return
        
        try:
            self._running = True
            ssl_config = self._get_ssl_config()
            
            config = uvicorn.Config(
                self.app,
                host=self.host,
                port=self.port,
                log_level="info",
                **ssl_config
            )
            
            self._server = uvicorn.Server(config)
            
            def run_server():
                try:
                    asyncio.set_event_loop(asyncio.new_event_loop())
                    loop = asyncio.get_event_loop()
                    loop.run_until_complete(self._server.serve())
                except Exception as e:
                    logger.error(f"Server error: {e}", exc_info=True)
                finally:
                    self._running = False
            
            self._server_thread = threading.Thread(target=run_server, daemon=True)
            self._server_thread.start()
            
            protocol = "https" if ssl_config else "http"
            logger.info(f"VR server started at {protocol}://{self.host}:{self.port}")
            
        except Exception as e:
            self._running = False
            logger.error(f"Failed to start VR server: {e}", exc_info=True)
            raise

    @rpc
    def stop(self):
        """Stop the VR teleoperation server."""
        if not self._running:
            logger.info("VR server not running")
            return
        
        try:
            self._running = False
            
            if self._server:
                self._server.should_exit = True
            
            if hasattr(self, '_server_thread') and self._server_thread:
                self._server_thread.join(timeout=5)
                if self._server_thread.is_alive():
                    logger.warning("Server thread did not stop gracefully")
            
            logger.info("VR server stopped")
            
        except Exception as e:
            logger.error(f"Error stopping VR server: {e}", exc_info=True)

    @rpc
    def get_stats(self):
        """Get server statistics."""
        return {
            "running": self._running,
            "frames_received": self.stats["frames_received"],
            "errors": self.stats["errors"],
            "last_frame_timestamp": self.stats["last_frame"].timestamp if self.stats["last_frame"] else None,
            "host": self.host,
            "port": self.port
        }

    @rpc
    def generate_certificate(self, cert_dir: str = "dimos/vr/certificates"):
        try:
            from ..generate_cert import generate_self_signed_cert
            
            cert_path = Path(cert_dir)
            cert_path.mkdir(exist_ok=True)
            
            cert_file = cert_path / "cert.pem"
            key_file = cert_path / "key.pem"
            
            generate_self_signed_cert(
                cert_file=str(cert_file),
                key_file=str(key_file)
            )
            
            return {
                "cert_file": str(cert_file.absolute()),
                "key_file": str(key_file.absolute()),
                "message": "SSL certificate generated successfully"
            }
            
        except Exception as e:
            logger.error(f"Failed to generate certificate: {e}", exc_info=True)
            return {"error": str(e)}

    def _close_module(self):
        """Cleanup when module is closed."""
        try:
            self.stop()
        except:
            pass
        super()._close_module()

    def __str__(self):
        return f"MetaQuestModule(port={self.port}, running={self._running})"
