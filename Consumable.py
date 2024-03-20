import sys
# relative path to folder which contains the Sandbox module
sys.path.insert(1, '../../')
from Sandbox_V1_3 import *

from enum import Enum

# using an enum probably makes less sense in Python than it does in some other languages, as Python allows you to do
# pretty much anything you like to a variable, at any time you like, but the general idea is to use an enum to define
# and stick to a finite and constant set of values
class Consumables(Enum):
    food = 0
    poison = 1
    water = 2

# A consumable object, which can be placed in the environment, can be food, water or poison, and can be detected by
# LightSensors as it has a LightSource attached.
class Consumable(System):

    # construct consumable
    def __init__(self, x, y, radius=0.5, quantity=10, recovery_time=10, type=Consumables.food):
        super().__init__(x, y)  # call System constructor, to allow for the possibility that a Consumable will move
        self.type = type
        # set colour according to type of consumable
        self.initial_type = type
        self.set_colour()
        self.stimulus = LightSource(x=x, y=y, colour=self.colour, label=self.colour)  # construct LightSource
        self.quantity = quantity  # the quantity determines how much of an effect consuming the item has on a HungryRobot
        self.recovery_time = recovery_time  # when an item is consumed, it can reappear when the recovery time expires. if you don't want it to recover, then just make this time longer than your simulation duration
        self.depleted = False  # initially, a Consumable is not depleted. When it is consumed, it is depleted and will be invisible until (and if) it recovers
        self.time_since_consumed = 0  # used to track time to recover
        self.radius = radius  # this is the radius within which a HungryRobot will consume it

    # step consumable. Consumables are stepped in order to implement recovery from depletion
    def step(self, dt):
        super().step(dt)  # call System step method, to allow for the possibility that a Consumable will move
        if self.depleted: # if the Consumable has been depleted, then wait for recovery_time to replenish and make it detectable again
            if self.time_since_consumed >= self.recovery_time:  # if consumable has reached recovery_time
                self.depleted = False  # replenish consumable
                self.stimulus.is_on = True  # make consumable detectable again
            else:
                self.time_since_consumed += dt  # increment time since consumable was depleted

        # self.stimulus.label = self.colour
        self.stimulus.colour = self.colour

    # when a HungryRobot passes within the Consumable's radius, it calls this method so that the resource is depleted
    def consume(self):
        if self.depleted:  # if already depleteddepleted, return zero
            return 0
        else:  # if not already depleted, return the quantity which determines how much of an effect will be had on the robot
            self.depleted = True  # set to depleted
            self.stimulus.is_on = False  # turn LightSource off, to make the Consumable invisible
            self.time_since_consumed = 0
            return self.quantity

    # draw consumable in the specified matplotlib axes
    def draw(self, ax):
        self.set_colour()
        alpha = 1
        if self.depleted:
            alpha = 0.3
        ax.add_artist(mpatches.Circle((self.x, self.y), self.radius, color=self.color, alpha=alpha))
        ax.plot(self.x, self.y, 'k.')

    # set colour of consumable for drawing
    def set_colour(self):
        if self.type == Consumables.food:
            self.colour = 'darkgreen'
        elif self.type == Consumables.water:
            self.colour = 'blue'
        elif self.type == Consumables.poison:
            self.colour = 'darkred'

    # draw consumable in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        self.set_colour()
        width = 0
        if self.depleted:
            width = 2
        pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color=self.colour, width=width, radius=scale*self.radius)

    def reset(self) -> None:

        super().reset()
        self.type = self.initial_type
        self.depleted = False  # initially, a Consumable is not depleted. When it is consumed, it is depleted and will be invisible until (and if) it recovers
        self.time_since_consumed = 0  # used to track time to recover
        self.stimulus.reset()
