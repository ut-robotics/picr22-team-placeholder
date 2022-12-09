import json
import time
import queue
import multiprocessing as mp
import websocket as wsc


class Referee:
    """Class for getting data from the referee server and making it accessible for the robot."""

    def __init__(self, robot_data):
        self.robot_data = robot_data
        self.queue = mp.Queue()

    def start(self):
        """Start the referee process"""
        self.running = True
        self.ws = wsc.WebSocket()
        self.connect()
        self.process = mp.Process(target=self.listen, args=())
        self.process.start()
        self.robot_data.logger.log.info("--REFEREE-- Started process!")

    def close(self):
        """Close the referee process"""
        self.running = False
        self.process.join()
        self.process.close()
        self.ws.close()
        self.robot_data.logger.log.info("--REFEREE-- Closed process!")

    def connect(self):
        """Connect to the referee server"""
        while True:  # Attempt reconnecting while the code is running
            try:
                self.ws.connect(self.robot_data.referee_ip)
                return True
            except ConnectionRefusedError:
                time.sleep(1)
                continue
            except OSError:
                self.robot_data.log.error("No route to referee server host. Make sure the IP is correct and the server is running. Retrying...")
                time.sleep(1)
                continue
            except:
                self.robot_data.logger.log.exception('')

    def listen(self):
        """Listen and add referee commands to queue"""
        self.robot_data.logger.log.info("--REFEREE-- Listening to commands.")
        while self.running:
            try:
                msg = self.ws.recv()
                try:
                    msg = json.loads(msg)
                    if self.robot_data.name in msg["targets"]:
                        self.queue.put(msg)
                except json.JSONDecodeError:
                    self.robot_data.logger.log.error("--REFEREE-- Received non-json message!")
                    continue
            except wsc.WebSocketConnectionClosedException:
                self.robot_data.logger.log.warning("--REFEREE-- Connection lost, reconnecting...")
                if self.connect():
                    self.robot_data.logger.log.info("--REFEREE-- Reconnected.")
                    continue
                else:
                    self.robot_data.logger.log.error("--REFEREE-- Failed to reconnect.")
                    self.close()
            except KeyboardInterrupt:
                self.robot_data.logger.log.info("--REFEREE-- Closing...")
                break
            except:
                self.robot_data.logger.log.exception('')   

    def get_cmd(self):
        """Attempt to get a command from queue"""
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None
