import asyncio
import websockets
import serial
import time
from threading import Thread

async def handler(websocket):
    try:
        async for message in websocket:
            print(f"Received: {message}")

            
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Client disconnected: {e}")
    except asyncio.CancelledError:
        print("Receive task was cancelled.")
    except Exception as e:
        print(f"Unexpected error: {e}")




# Start the WebSocket server
async def main():
    server = await websockets.serve(handler, "192.168.1.75", 8000, ping_interval=30, ping_timeout=20)
    print("WebSocket server started on ws://192.168.1.75:8000")
    await server.wait_closed()

# Run the event loop
asyncio.run(main())
