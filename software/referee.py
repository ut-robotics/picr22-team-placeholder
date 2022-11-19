import json
import time
import queue
import multiprocessing as mp
import websocket as wsc


class Referee:
    """Class for getting data from the referee server and making it accessible for the robot."""

    def __init__(self, robot_data):
        self.ip = robot_data.referee_ip
        self.name = robot_data.name
        self.queue = mp.Queue()

    def start(self):
        """Start the referee process"""
        self.running = True
        self.ws = wsc.WebSocket()
        self.connect()
        self.process = mp.Process(target=self.listen, args=())
        self.process.start()
        print("--REFEREE-- Started process!")

    def close(self):
        """Close the referee process"""
        self.running = False
        self.process.join()
        self.process.close()
        self.ws.close()
        print("--REFEREE-- Closed process!")

    def connect(self):
        """Connect to the referee server"""
        for _ in range(15):  # Attempt reconnecting for ~15 seconds before giving up
            try:
                self.ws.connect(self.ip)
                return True
            except ConnectionRefusedError:
                print("--REFEREE-- Retrying...")
                time.sleep(1)
                continue
        return False

    def listen(self):
        """Listen and add referee commands to queue"""
        print("--REFEREE-- Listening to commands.")
        while self.running:
            try:
                msg = self.ws.recv()
                try:
                    msg = json.loads(msg)
                    if self.name in msg["targets"]:
                        self.queue.put(msg)
                except json.JSONDecodeError:
                    print("--REFEREE-- Received non-json message!")
                    continue
            except wsc.WebSocketConnectionClosedException:
                print("--REFEREE-- Connection lost, reconnecting...")
                if self.connect():
                    print("--REFEREE-- Reconnected.")
                    continue
                else:
                    print("--REFEREE-- Failed to reconnect.")
                    self.close()
            except KeyboardInterrupt:
                print("--REFEREE-- Closing...")
                break

    def get_cmd(self):
        """Attempt to get a command from queue"""
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None
