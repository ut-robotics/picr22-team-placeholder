# NOTE - THIS IS WIP, to be merged with the rest of the code soon tm
from time import time


def Searching(self):
    """State for searching for the ball"""
    if self.ball_count != 0:
        self.current_state = State.BallFound
    else:
        if time() < self.search_end:
            print("--Searching-- Moving to look for ball")
            self.robot.move(0, 0, self.search_speed, 0)
        else:
            print("--Searching-- Entering wait to scan surroundings")
            self.current_state = State.Wait
            self.wait_end = time() + self.scan_wait_time
            self.next_state = State.Searching


def Wait(self):
    """State for waiting"""
    if self.next_state == State.Searching and self.ball_count != 0:
        print("--Wait-- Found ball.")
        self.current_state = State.BallFound
        self.next_state = None
        return
    if time() >= self.wait_end:
        if self.next_state != None:
            self.current_state = self.next_state
        else:
            self.current_state = State.Searching
        self.next_state = None
        if self.current_state == State.Searching:
            self.search_end = time() + self.scan_move_time


def BallAlign(self):
    """State for aligning the robot with the ball's direction"""
    if self.no_balls_frames > self.max_ball_miss:  # lost the ball, TODO - increment the value
        self.current_state, self.search_end = self.back_to_search()
        return
    print("--BallAlign-- Ball found.")
    if self.ball.x > (self.middle_point + self.camera_deadzone):
        print("--BallAlign-- right")
        self.robot.move(0, 0, -self.max_speed, 0)
    elif self.ball.x < (self.middle_point - self.camera_deadzone):
        print("--BallAlign-- left")
        self.robot.move(0, 0, self.max_speed, 0)

    else:
        self.current_state = State.DriveToBall


def DriveToBall(self):
    """State for driving to the ball."""
    if self.no_balls_frames > self.max_ball_miss:  # lost the ball, TODO - increment the value
        self.current_state, self.search_end = self.back_to_search()
        return
    print("--DriveToBall-- Driving to ball.")
    if self.ball.distance < self.min_distance - 20:
        print("--DriveToBall-- ball too close, distance:", self.ball.distance)
        self.robot.move(0, -self.max_speed, 0)
    elif self.ball.distance > self.min_distance:
        print("--DriveToBall-- ball far, distance:", self.ball.distance)
        self.robot.move(0, self.max_speed, 0)
    else:
        self.current_state = State.Orbiting


def Orbiting(self):
    """State for orbiting around the ball and trying to find the basket."""
    if self.basket_color == Color.MAGENTA:
        basket = self.processedData.basket_m
    elif self.basket_color == Color.BLUE:
        basket = self.processedData.basket_b
        # TODO - adjust based on the ball
    if basket.exists:
        if basket.x < self.middle_point - 5:  # left
            self.robot.move(self.max_speed*0.15, 0, self.max_speed*0.5)
        elif basket.x > self.middle_point + 5:  # right
            self.robot.move(-self.max_speed*0.15, 0, -self.max_speed*0.5)
        else:
            self.current_state = State.BallThrow
    else:
        # move faster to try and find the basket
        self.robot.move(0.5, 0, 1.5)

# TODO - implement


def BasketBallAlign(self):
    """State for aligning the ball and the basket."""
    print("Unimplemented state BasketBallAlign.")


def BallThrow(self):
    """State for throwing the ball into the basket"""
    if self.basket_color == Color.MAGENTA:
        basket = self.processedData.basket_m
    elif self.basket_color == Color.BLUE:
        basket = self.processedData.basket_b
    if basket.exists:  # TODO
        print("--BallThrow-- Throwing ball, basket distance:",
              basket.distance)
    elif self.ball_count != 0:
        print("--BallThrow-- No basket, going back to orbiting.")
        current_state = State.Orbiting
    else:
        print("--BallThrow-- No basket or ball, going back to throwing.")
        current_state = State.Searching


def RemoteControl(self):
    if not self.controller.is_remote_controlled():
        self.current_state, self.search_end = self.back_to_search()
