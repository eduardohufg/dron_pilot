# ws_client_img.py
import asyncio
import websockets
import cv2
import base64
import numpy as np

URL = "ws://localhost:8000/ws/connection/img"

async def receive_images():
    async with websockets.connect(URL) as websocket:
        print("✅ Conectado al WebSocket")
        try:
            while True:
                base64_data = await websocket.recv()

                # Decodificamos de base64 a imagen
                img_data = base64.b64decode(base64_data)
                nparr = np.frombuffer(img_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is not None:
                    cv2.imshow("Stream desde ROS2", frame)

                # Salir si se presiona ESC
                if cv2.waitKey(1) == 27:
                    break

        except websockets.ConnectionClosed:
            print("🔌 Conexión cerrada")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_images())
