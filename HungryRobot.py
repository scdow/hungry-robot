import sys
# relative path to folder which contains the Sandbox module
sys.path.insert(1, '../../')
from Sandbox_V1_3 import *

from Consumable import *

'''
	The implementation of this sensor is a bit odd, but it is here for use by
	HungryRobots, which may need to adapt their behaviours depending on how close to
	death (hungry) they are. in general, an organism or human machine will not have
	direct knowledge of its energy level, and some kind of sensor(s) will need to be
	used to apprehend it.

	The reason i don't especially like the implementation of this is that the
	robot's energy level is a property of the robot, and the sensor detects that
	property for use in the robot's controller. technically, this results in a
	circular reference, which I would normally avoid, but for now it works as
	desired, so it will be fixed in later implementations but is fine for now.

	The advantage of implementing this as a Sensor is that we can attach different
	kinds of NoiseSources to it, to model various kinds of imperfect sensing.
'''
class RobotEnergySensor(Sensor):

    # construct energy sensor
    def __init__(self, robot, x, y, noisemaker=None):
        super().__init__(x=x, y=y)
        self.activation = 0  # sensor activation. this variable is updated in and returned from the step method. it is stored separately in case you want to access it multiple times between simulation steps, although that is unlikely to be necessary
        self.activations = [self.activation]  # for plotting and analysis, a sensor keeps a complete record of its activation over time
        self.noisemaker = noisemaker
        self.robot = robot

    # step sensor
    def step(self, dt):
        super().step(dt)
        self.activation = self.robot.energy  # get robot's energy level

        # add noise, if a noisemaker is implemented
        if self.noisemaker != None:
            self.activation += self.noisemaker.step(dt)

        self.activations.append(self.activation)  # store sensor activation

        return self.activation  # return sensor activation

    ''''
        A method to reset a sensor to its initial state, by resetting its ``x``, ``y``, ``theta``, ``activation``, and ``activations`` attributes to their values at time of construction. If the sensor has a ``noisemaker``, then the reset method of that object will also be called.
	'''
    def reset(self):
        super().reset()
        self.activation = self.activations[0]
        self.activations = [self.activation]
        if self.noisemaker:
            self.noisemaker.reset()

    # A method to get the sensor's data, in the form of a dict.
    def get_data(self) -> dict:
        data = super().get_data()
        data["activations"] = self.activations
        data["noises"] =  None
        if self.noisemaker:
            data["noises"] = self.noisemaker.get_data()["noises"]
        return data

'''
	A subclass of Robot, which can sense and consume food and poison objects
	(Consumables), and which gains and loses energy in response to consuming them.

	An energy level sensor is implemented, although it is not used. The reason
	for implementing this is: the energy level might make a good choice for
	essential variable (if you want to use one), and it could be made one of the
	inputs to a controller.

	For biological and human-made agents, it is not possible to have perfect
	knowledge of internal states such as energy levels. therefore, it is more
	interesting and plausible if energy level input to the controller comes
	from a sensor with some level and type of noise applied to it than not.
'''
class HungryRobot(Robot):
    # construct robot
    def __init__(self, x, y, controller,
                 left_food_sources,
                 right_food_sources,
                 left_poison_sources,
                 right_poison_sources,
                 consumables, radius=1, theta=0,
                 initial_energy=100,
                 maximum_energy=200,
                 left_food_sensor_angle=np.pi/4,
                 right_food_sensor_angle=-np.pi/4,
                 left_food_noisemaker=None,
                 right_food_noisemaker=None,
                 food_field_of_view=2*np.pi,
                 left_poison_sensor_angle=np.pi/4,
                 right_poison_sensor_angle=-np.pi/4,
                 left_poison_noisemaker=None,
                 right_poison_noisemaker=None,
                 poison_field_of_view=2*np.pi,
                 decay_rate=1, decay_rate2=0.01,
                 max_speed=1, energy_sensor_noisemaker=None,
                 left_motor_noisemaker=None,
                 right_motor_noisemaker=None,
                 left_motor_max_speed=2,
                 right_motor_max_speed=2,
                 left_motor_inertia=0,
                 right_motor_inertia=0,
                 left_motor_reversed=False,
                 right_motor_reversed=False
                 ):

        left_poison_sensor = LightSensor(light_sources=left_poison_sources, x=x, y=y, noisemaker=left_poison_noisemaker,
                                       FOV=poison_field_of_view, label='darkred')  # construct left poison sensor. at this point, dummy positions are given for light sensors. they will be fixed when the super constructor is called
        right_poison_sensor = LightSensor(light_sources=right_poison_sources, x=x, y=y, noisemaker=right_poison_noisemaker,
                                        FOV=poison_field_of_view, label='darkred')  # construct right poison sensor

        left_food_sensor = LightSensor(light_sources=left_food_sources, x=x, y=y, noisemaker=left_food_noisemaker,
                                       FOV=food_field_of_view, label='darkgreen')  # construct left poison sensor. at this point, dummy positions are given for light sensors. they will be fixed when the super constructor is called

        right_food_sensor = LightSensor(light_sources=right_food_sources, x=x, y=y, noisemaker=right_food_noisemaker,
                                       FOV=food_field_of_view, label='darkgreen')  # construct left poison sensor. at this point, dummy positions are given for light sensors. they will be fixed when the super constructor is called

        # keep a list of these for potentially disturbing later
        self.poison_sensors = [left_poison_sensor, right_poison_sensor]

        # keep a list of these for potentially disturbing later
        self.food_sensors = [left_food_sensor, right_food_sensor]

        sensor_angles = [left_food_sensor_angle, right_food_sensor_angle, left_poison_sensor_angle, right_poison_sensor_angle]

        sensors = [left_food_sensor, right_food_sensor, left_poison_sensor, right_poison_sensor]

        self.energy_sensor = RobotEnergySensor(robot=self, x=x, y=y, noisemaker=energy_sensor_noisemaker)  # there is a circular reference between robot and sensor - not ideal, but seems okay here
        self.consumables = consumables  # a list of consumables which will affect this Robot
		# construct Robot
        super().__init__(x=x, y=y,
                         controller=controller,
                         radius=radius, theta=theta,
                         sensors=sensors,
                         sensor_angles=sensor_angles,
                         left_motor_noisemaker=left_motor_noisemaker,
                         right_motor_noisemaker=right_motor_noisemaker,
                         left_motor_max_speed=left_motor_max_speed,
                         right_motor_max_speed=right_motor_max_speed,
                         left_motor_inertia=left_motor_inertia,
                         right_motor_inertia=right_motor_inertia,
                         left_motor_reversed=left_motor_reversed,
                         right_motor_reversed=right_motor_reversed
                         )  # call Robot constructor. The LightSensors already implemented in Robot function as food sensors

        left_food_sensor.color = 'darkgreen'  # set food sensor colours. poison sensor colours are left at the default, which is red
        right_food_sensor.color = 'darkgreen'
        self.energy_sensor.color = 'yellow'  # set energy level sensor
        self.energy = initial_energy  # set initial energy level
        self.energies = [initial_energy]  # store energy level
        self.decay_rate = decay_rate  # rate at which energy decays when used by motors
        self.decay_rate2 = decay_rate2  # rate at which energy decays even if the motors are inactive
        self.alive = True
        self.maximum_energy = maximum_energy
        self.f_eat = 0
        self.p_eat = 0
        self.survivla_time = 0

    # step robot
    def step(self, dt):
        super().step(dt)  # call Robot's step function. note that the control method (below) gets called from there
        if self.alive:
            self.survivla_time += 1
            for consumable in self.consumables:
                if np.linalg.norm([self.x-consumable.x, self.y-consumable.y]) < consumable.radius:   # l2 norm
                    quantity = consumable.consume()
                    if consumable.type == Consumables.food:
                        if quantity > 0 :
                            self.f_eat += 1
                        self.energy += quantity
                    elif consumable.type == Consumables.poison:
                        if quantity >0 :
                            self.p_eat += 1
                        self.energy -= quantity
            self.energy = max(self.energy, 0) # prevent energy falling below zero
            self.energy = min(self.energy, self.maximum_energy) # prevent energy going over "full"
        self.energies.append(self.energy)

    # this is separated from the step method as it is easier to override in any subclasses of Robot than step, which
    def control(self, activations: List[float], dt: float):

        # update all sensor measurements
        activations.append(self.energy_sensor.step(dt))

        # get motor speeds from controller
        left_speed, right_speed = self.controller.step(dt, activations)

        if self.alive:
            # update energy. the faster the robot's wheels turn, the quicker it loses energy
            self.energy -= np.abs(left_speed) * dt * self.decay_rate
            self.energy -= np.abs(right_speed) * dt * self.decay_rate
            self.energy -= dt * self.decay_rate2  # some energy is lost even if the robot does not move
            self.energy = max(self.energy, 0)  # prevent energy falling below zero

        if self.energy <= 0:
            self.alive = False
            left_speed = 0
            right_speed = 0

        return left_speed, right_speed

    # draw robot in the specified matplotlib axes
    def draw(self, ax):
        # call draw from super to draw Robot
        super().draw(ax)

    # draw robot in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        # call draw from super to draw Robot
        super().pygame_draw(screen, scale, shiftx, shifty)
        radius = self.radius * self.energy / self.maximum_energy
        pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color='yellow', width=2, radius=scale*radius)

    def reset(self, reset_energy: bool=True, reset_controller: bool=True) -> None:
		# reset Robot
        super().reset(reset_controller)
		# reset energy level and sensor
        if reset_energy:
            self.energy = self.energies[0]
            self.energies = [self.energy]
            self.energy_sensor.reset()
            self.alive = True

    def get_data(self):

        data = super().get_data()

		# not currently used, but plotting code relies on classname
		# being equal to "Robot", so I have added subclassname here
        data["subclassname"] = "HungryRobot"
        data['survival_time'] = self.survivla_time
        data['f_eat'] = self.f_eat
        data['p_eat'] = self.p_eat
        data["energy"] = self.energies
        data["energy_sensor"] = self.energy_sensor.get_data()

        return data
