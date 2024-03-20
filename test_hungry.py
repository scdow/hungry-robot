import sys
# relative path to folder which contains the Sandbox module
sys.path.insert(1, '../../')
from Sandbox_V1_3 import *

# from controllers import *
# from initialisers import *
# from phase_portrait import *

from Consumable import *
from HungryRobot import *
from controllers import *
from disturbances import *

def run_once():


    # #########################################################################
    # #                           Noise section                               #
    # #########################################################################
    #
    # # noise parameters for the robot's left motor
    # max_white_noise = 0. # try 0.1 and -0.1 at first for white noise params
    # min_white_noise = -0.
    # max_brown_noise_step = 0. # try 0.005 at first
    # spike_noise_prob = 0.025 # try 0.025 at first
    # pos_spike_size = 1 # if the prob param is 0, there are no spikes, so these params have no effect
    # neg_spike_size = -1
    #
    # # construct noise source for robot's left motor
    # left_motor_noisemaker = NoiseMaker(white_noise_params=[max_white_noise, min_white_noise], brown_noise_step=max_brown_noise_step, spike_noise_params=[spike_noise_prob, pos_spike_size, neg_spike_size])
    #
    # # we can quickly enable/disable noise by uncommenting or commenting out this line
    # left_motor_noisemaker = None
    #

    # energy_sensor_noisemaker = WhiteNoiseSource(min_val=-5, max_val=5)
    energy_sensor_noisemaker = None
    left_food_noisemaker=None # WhiteNoiseSource(min_val=-0.5, max_val=0.5)
    right_food_noisemaker=None
    left_poison_noisemaker=None
    right_poison_noisemaker=None

    #########################################################################
    #                        Prepare simulation                             #
    #########################################################################

    food_num = 15
    poison_num = 10
    scale = 10

    foods_and_poisons = []
    foods_stimuli = []  # list of LightSources attached to food consumables, used by the robot's food sensors
    poisons_stimuli = []  # list of LightSources attached to food consumables, used by the robot's poison sensors
    # generate food items
    for i in range(food_num):
        x = random_in_interval(-scale, scale)
        y = random_in_interval(-scale, scale)
        food = Consumable(x, y, radius=1, recovery_time=50, quantity=25)
        foods_and_poisons.append(food)
        foods_stimuli.append(food.stimulus)
    # generate poison items
    for i in range(poison_num):
        x = random_in_interval(-scale, scale)
        y = random_in_interval(-scale, scale)
        poison = Consumable(x, y, radius=1, recovery_time=50, quantity=25, type=Consumables.poison)
        foods_and_poisons.append(poison)
        poisons_stimuli.append(poison.stimulus)

    # hand-coded controller
    # controller=HungryController(step_fun=hungry_stepfun, gain=2)
    controller=HungryController(step_fun=hungry_stepfun2, gain=2)

    # dynamical controller
    # run_evolution()
    # controller = CTRNN_Controller(inputs_n=2, commands_n=2, gain=gain, genome=genome, config=config, time_constant=time_constant, noisemakers=None, noisemakers_inds=None)

    # create robot
    robot = HungryRobot(x=0, y=0, controller=controller,
    left_food_sources=foods_stimuli, # the lights attached to food items
    right_food_sources=foods_stimuli,
    left_poison_sources=poisons_stimuli, # the lights attached to poison items
    right_poison_sources=poisons_stimuli,
    left_food_sensor_angle=np.pi/3, # i only put a small difference between sensor pair angles so that we can see them all
    right_food_sensor_angle=-np.pi/3,
    left_poison_sensor_angle=np.pi/5,
    right_poison_sensor_angle=-np.pi/5,
    food_field_of_view=0.8*np.pi, # sensor angular fields of view
    poison_field_of_view=0.8*np.pi,
    consumables=foods_and_poisons, # all of the consumables
    energy_sensor_noisemaker=energy_sensor_noisemaker,
    left_food_noisemaker=left_food_noisemaker,
    right_food_noisemaker=right_food_noisemaker,
    left_poison_noisemaker=left_poison_noisemaker,
    right_poison_noisemaker=right_poison_noisemaker,
    decay_rate=2, # determines the rate at which motion costs energy
    decay_rate2=0.01) # some energy is lost even if the robot does not move

    # #########################################################################
    # #                      Simulation parameters                            #
    # #########################################################################

    # simulation run parameters
    screen_width = 800 # height and width of animation window, in pixels
    duration = 300 # number of simulation time units to simulate for
    n_runs = 1 # number of simulations to run
    animate = True # whether or not to animate
    animation_frame_delay = 1 # the 10s of milliseconds to pause between frames
    #
    #########################################################################
    #                       Simulation section                              #
    #########################################################################

    # put robot into list of agents for Simulator
    agents=[robot]
    """set the robot initial poses"""
    def init_fun(self):
        # circle radius
        rad = 10
        # number of points
        n = n_runs-1
        # generate angles
        thetas = np.linspace(0, 2 * np.pi, num=n, endpoint=False)
        # get angle index. by using the modulo operator, we ensure that this index
        # will always be valid, even if the function is used for more than n runs
        if self.init_ind == 0:
            self.push(x=self.x,y=self.y)
        else:
            new_init_ind = self.init_ind-1
            ind = new_init_ind % n
            # ind = self.init_ind % n
            # print(self.init_ind, new_init_ind, n, ind)
            # get angle
            angle = thetas[ind]
            # use angle to generate the robot's pose: [x, y, theta]
            # - it is important to use the push method for this, so that the histories
            #	of the adjusted variables get updated correctly
            self.push(x=rad * np.cos(angle), y=rad * np.sin(angle), theta=angle + np.pi)

    """A perturb function for perturbing the robot's pose."""
    # add randomness to robot's initial locations & orientations
    def perturb(self):
        x = self.x + random_in_interval(minimum=-10, maximum=10)
        y = self.y + random_in_interval(minimum=-10, maximum=10)
        theta = self.theta + random_in_interval(minimum=-math.pi/4, maximum=math.pi/4)
        print(x,y,theta)
        self.push(x=x, y=y, theta=theta)

    # robot.init_fun = init_fun
    """make the robot inital location and orientation randomly"""
    robot.perturb_fun = perturb

    # this disturbance turns food in to poison and vice versa
    # - a successful robot will be able to adapt to this change
    switcheroo = FoodSwitcherooDisturbanceSource(start_times=[100, 200], consumables=foods_and_poisons, enabled=True)
    # disturbances = [switcheroo]
    disturbances = []

    # get Simulator object - note that agents, environmental systems, and disturbances are all passed to its constructor
    sim = Simulator(agents=agents, envs=foods_and_poisons+foods_stimuli+poisons_stimuli, duration=duration, dt=0.1, disturbances=disturbances)
    # get SimulationRunner object
    sim_runner = SimulationRunner(sim, animate=animate, pause_ani=True, animation_delay=animation_frame_delay, screen_width=screen_width)
    # Run simulation n times
    sim_data = sim_runner.run_sims(n=n_runs)


    ########################################################################
    #                     Plotting and analysis section                     #
    #########################################################################

    # plot agents' trajectories
    ax = plot_all_sims_trajectories(sim_data, show_cbar=True, cbar_fig=False)
    for light in foods_stimuli+poisons_stimuli:
        light.draw(ax)
    # plot robots' basic data (plots of robot's various noisemakers are not produced by this function call)
    plot_all_robots_basic_data(sim_data, multiple_plots=False, show_motors=True, show_controllers=True, show_sensors=True)
    # plot noise
    plot_all_robots_noise(sim_data)

    # plot robot's energy over time for every simulation run
    for i, data in enumerate(sim_data):
        ts = data["ts"]
        robot_data = data["agents"][0]
        plt.figure()
        plt.plot(ts, robot_data["energy_sensor"]["activations"])
        plt.title("Robot energy sensor over time, run " + str(i))
        plt.xlabel("t")
        plt.ylabel("Energy")

    print('Energy:',robot_data["energy_sensor"]["activations"][-1])
    print('Survival time steps:',robot_data['survival_time'] ,'over', duration/0.1 )
    print('Eat food:',robot_data['f_eat'])
    print('Eat poison',robot_data['p_eat'])
    # evalution indicator = 1/4 * survival_time/duration_time + 1/4 * energy/max_energy + 1/2* (food eat - poison eat)/ (max food eat +max poison eat)
    eval_indicator = 1/4* robot_data['survival_time']/3000 \
                     + 1/4* robot_data["energy_sensor"]["activations"][-1]/200 \
                     + 1/2*(robot_data['f_eat']-robot_data['p_eat'])/(90+60)
    print('Adaptivity score:', eval_indicator)
    plt.show()


if __name__ == '__main__':
    for i in range(5):
        run_once()

