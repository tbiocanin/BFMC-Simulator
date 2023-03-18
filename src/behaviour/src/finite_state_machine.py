#!/usr/bin/env python3

from enum import Enum, auto

class States(Enum):
    # State Class, Defines the car states
    IDLE = auto()
    DRIVE = auto()
    STOP = auto()
    INTERSECTION = auto()
    PARKING = auto()
    OVERTAKING = auto()

class CarState():
    def __init__(self):
        self.__state = States.IDLE

    def set_state(self, state):
        self.__state = state

    def get_state(self):
        # print("Iz masine stanja skripta", self.__state)
        return self.__state

