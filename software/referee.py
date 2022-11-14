import asyncio
import websockets
import json
from states import State
from Color import Color
class Referee:
    """Class for getting messages from referee server"""
    def __init__(self, robot_data):
        self.robot_data = robot_data
        asyncio.ensure_future(self.monitor())
    
    # TODO - implement actual states for all the stuff
    async def monitor(self):
        while True:
            try:
                async with websockets.connect(self.robot_data.referee_ip) as websocket:
                    while True:
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
                            self.robot_data.current_state = State.Searching
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