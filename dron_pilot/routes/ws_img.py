# ws_img.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio

from ..ros2_bridge import get_ros2_node
from ..app_globals import globals
img_router = APIRouter()
active_connections: Set[WebSocket] = set()

async def send_image_to_websockets(frame_base64: str):
    disconnected = set()
    for ws in active_connections.copy():
        try:
            await ws.send_text(frame_base64)
        except Exception as e:
            print(f"Error sending image: {e}")
            disconnected.add(ws)
    active_connections.difference_update(disconnected)

def register_image_callback():
    node = get_ros2_node()
    if node is None:
        raise RuntimeError("ROS2 node not initialized!")

    def on_image(frame_b64: str):
        asyncio.run_coroutine_threadsafe(
            send_image_to_websockets(frame_b64), globals.loop
        )

    node.register_image_callback(on_image)

@img_router.websocket("/img")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            # Mantener la conexión
            await asyncio.sleep(999999999)
    except WebSocketDisconnect:
        print("Client disconnected from /img")
    finally:
        active_connections.discard(websocket)
