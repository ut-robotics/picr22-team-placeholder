import asyncio
from threading import Thread
import websockets
import json
from states import State
from Color import Color
from time import time

class RefereeBackend:
    """Class for getting messages from referee server"""
    running = True
    def __init__(self, robot_data): # TODO - maybe figure out a cleaner and more robust way, but this does technically work
        self.robot_data = robot_data
        print("the monitor")
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.monitor()) 
        #loop.close()
        
    # TODO - implement actual states for all the stuff
    async def monitor(self):
        while self.running:
            print("Connecting...")
            try:
                async with websockets.connect(self.robot_data.referee_ip) as websocket:
                    print("Connected to server!")
                    while self.running:
                        data = json.loads(await websocket.recv())
                        if self.robot_data.name in data["targets"]:
                            robot_index = data["targets"].index(self.robot_data.name)
                            print("our robot found, index:", robot_index)
                        else:
                            robot_index = None
                            continue # don't do anything if our robot is not on list

                        if data["signal"] == "start":
                            basket_str = data["baskets"][robot_index]
                            if basket_str == "blue":
                                basket = Color.BLUE
                            elif basket_str == "magenta":
                                basket = Color.MAGENTA
                            else:
                                raise ValueError("Unknown basket colour:", basket_str)
                            self.robot_data.basket_color = basket
                            self.robot_data.drive_end_time = time() + 3.5
                            self.robot_data.current_state = State.DriveToSearch
                            print("STARTING ROBOT, basket:", basket)

                        elif data["signal"] == "stop":
                            print("STOPPING ROBOT")
                            self.robot_data.current_state = State.Stopped
                        else:
                            raise ValueError("Unknown signal:", data["signal"])

            except(websockets.exceptions.ConnectionClosedError):
                print("Server closed. Attempting to reconnect...")
                continue
            except(ConnectionRefusedError):
                print("Connection refused, trying again..")
                continue
            except(KeyboardInterrupt):
                print("Closing websocket...")
                break
            
    def stop(self):
        """Stops listening for referee commands."""
        self.running = False
        
class Referee:
    """Class for enabling the use of a referee server to start and stop the robot."""

    def __init__(self, robot_data):
        self.robot_data = robot_data
        self.referee = None

    def start(self):
        """Starts referee listening in a separate thread"""
        self.thread = Thread(target=self.listen, args=()).start()

    def listen(self):
        """Listen for referee commands"""
        print("LISTENING")
        self.referee = RefereeBackend(robot_data=self.robot_data)

    def stop(self): # TODO - this is broken
        """Stops the referee server."""
        self.referee.stop()