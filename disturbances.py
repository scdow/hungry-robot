import sys
# relative path to folder which contains the Sandbox module
sys.path.insert(1, '../../')
from Sandbox_V1_3 import *

from Consumable import *


# THESE LINES APPLY TO A FEATURE WHICH HAS BEEN DISABLED FOR THIS CHALLENGE, BY BEING COMMENTED OUT - IF YOU UNCOMMENT
# THE LINE IN THE STEP FUNCTION WHICH DISTURBS THE ROBOT, THE EFFECT WILL BE AS DESCRIBED BELOW
# the disturbance also changes the decay_rate parameter of the given robot, a change made specifically for this challenge. when
# the consumables are food, the decay_rate determines the energetic cost of movement. When the consumables are poison,
# penalising movement in the usual way may be an issue - if there is a cost to movement, and a cost to consuming poison,
# then the best strategy to minimise costs may be to simply stop moving (the robot will still lose some energy, even
# when not moving, but this loss happens in either case).
# when the sign of decay_rate is inverted, energy grows when the robot moves, i.e. movement is  directly rewarded,
# so that the best strategy for the robot to maximise energy is to keep moving and avoid poison at the same time.
class FoodSwitcherooDisturbanceSource(DisturbanceSource):

    # construct disturbance source
    def __init__(self, start_times, enabled, robot=None, consumables=None):
        super().__init__(start_times, stop_times=[], enabled=enabled)
        if consumables:
            self.add_consumables(consumables)
        if robot:
            self.add_robot(robot)

    # add robot to DisturbanceSource - this allows for the DisturbanceSource to be created in advance of the Robot,
    # so that the same pattern of using a function to generate DisturbanceSources in advance can be followed as was
    # used in the ealier motor_inversion_challenge.py file
    def add_robot(self, robot):
        self.robot = robot

    # add consumables to DisturbanceSource - this allows for the DisturbanceSource to be created in advance of the
    # consumables, so that the same pattern of using a function to generate DisturbanceSources in advance can be
    # followed as was used in the ealier motor_inversion_challenge.py file
    def add_consumables(self, consumables):
        self.consumables = consumables

    # step disturbance source
    def step(self, dt):
        super().step(dt)
        if self.enabled:
            # self.robot.decay_rate = -self.robot.decay_rate  # switch between penalising and rewarding motion
            for consumable in self.consumables:  # switch consumable types
                if consumable.type == Consumables.food:
                    consumable.type = Consumables.poison
                elif consumable.type == Consumables.poison:
                    consumable.type = Consumables.food

            self.enabled = False  # this is a one-shot disturbance, so disable it again immediately

    def reset(self) -> None:
        """
            Reset disturbance, so it can be re-used.
        """
        super().reset()
        for consumable in self.consumables:  # switch consumable types
            consumable.reset()
