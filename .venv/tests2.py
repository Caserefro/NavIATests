import asyncio
import websockets
import serial
from threading import Thread
from functools import partial


class UARTManager:
    def __init__(self):
        self.uart_ports = {
            'UART2': {'device': '/dev/ttyAMA2', 'baudrate': 115200, 'timeout': 1},
            'UART3': {'device': '/dev/ttyAMA3', 'baudrate': 115200, 'timeout': 1},
            'UART4': {'device': '/dev/ttyAMA4', 'baudrate': 115200, 'timeout': 1}
        }
        self.serial_connections = {}
        self.running = False

    def initialize_ports(self):
        """Initialize all UART ports"""
        print("Initializing UART ports...")
        for name, config in self.uart_ports.items():
            try:
                self.serial_connections[name] = serial.Serial(
                    port=config['device'],
                    baudrate=config['baudrate'],
                    timeout=config['timeout']
                )
                print(f"{name} ({config['device']}) initialized")
            except Exception as e:
                print(f"Failed {name}: {str(e)}")
                self.serial_connections[name] = None

    def read_from_port(self, port_name):
        """Continuous UART read thread"""
        while self.running:
            if self.serial_connections[port_name] and self.serial_connections[port_name].in_waiting:
                try:
                    data = self.serial_connections[port_name].read(
                        self.serial_connections[port_name].in_waiting
                    )
                    print(f"[{port_name} RX] {data.decode().strip()}")
                except Exception as e:
                    print(f"[{port_name} ERROR] {str(e)}")

    def write_to_port(self, port_name, message):
        """Write to UART with validation"""
        if self.serial_connections.get(port_name):
            try:
                self.serial_connections[port_name].write(f"{message}\n".encode())
                print(f"[{port_name} TX] {message}")
            except Exception as e:
                print(f"[{port_name} WRITE ERROR] {str(e)}")
        else:
            print(f"[{port_name} NOT CONNECTED]")

    def start(self):
        """Start UART service"""
        self.initialize_ports()
        self.running = True
        # Start read threads
        for name in self.serial_connections:
            if self.serial_connections[name]:
                Thread(target=self.read_from_port, args=(name,), daemon=True).start()
        print("UART Manager ready")

    def stop(self):
        """Cleanup resources"""
        self.running = False
        for name, conn in self.serial_connections.items():
            if conn: conn.close()
        print("UART ports closed")


async def websocket_handler(websocket, uart_manager):
    try:
        async for message in websocket:
            print(f"Received command: {message}")
            uart_manager.write_to_port(port_name, message)
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Client disconnected: {e}")
    except Exception as e:
        print(f"WebSocket error: {str(e)}")


async def main():
    uart_manager = UARTManager()
    uart_manager.start()

    try:
        async def handler(websocket):
            await websocket_handler(websocket, uart_manager)

        server = await websockets.serve(
            handler,
            "0.0.0.0",
            8000,
            ping_interval=30,
            ping_timeout=20
        )
        print("WebSocket server started on ws://0.0.0.0:8000 (all interfaces)")
        await server.wait_closed()
    finally:
        uart_manager.stop()


if __name__ == "__main__":
    asyncio.run(main())
