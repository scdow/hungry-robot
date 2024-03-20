import sys
import os
import pickle
import math

import matplotlib.pyplot as plt
import numpy as np

# relative path to folder which contains the Sandbox module
sys.path.insert(1, '../../')
from Sandbox_V1_3 import *


from Consumable import *
from HungryRobot import *
from disturbances import *
import neat
import visualize

do_plots_ = False
animate_ = False


'''

    A controller which includes an evolved CTRNN.

    It is necessary to subclass Controller, rather than just using
    step_fun, as in simpler examples, because the CTRNN is an object,
    meaning that we need to make it an attribute of the controller.

'''
class CTRNN_Controller(Controller):

    # construct controller
    def __init__(self, inputs_n, commands_n, gain, genome, config, time_constant, noisemakers: List[NoiseSource]=None, noisemakers_inds=None):

        # construct CTRNN
        self.net = neat.ctrnn.CTRNN.create(genome, config, time_constant)
        self.time_constant = time_constant

        # these gain factors can be manipulated (e.g. by Disturbances)
        # or we can just use them to make the robot respond more/less
        # strongly to sensory input
        # self.l_gain, self.r_gain = gain
        self.lf_gain, self.rf_gain, self.lp_gain, self.rp_gain, self.e_gain = gain, gain, gain, gain, gain

        # call superclass (Controller) __init__
        super().__init__(inputs_n=inputs_n, commands_n=commands_n, noisemakers=noisemakers, noisemakers_inds=noisemakers_inds, step_fun=None)

    # step controller forwards in time
    def step(self, dt, inputs):

        # timer is not used in this class
        # self.t += dt  # increment time variable by simulation step size

        """inputs[0] = left_food_activation
        inputs[1] = right_food_activation
        inputs[2] = left_poison_activation
        inputs[3] = right_poison_activation
        inputs[4] = energy_activation
        # - not used in this basic controller, but a really good one certainly would pay attention to it!"""

        # amplify inputs
        inputs[0] *= self.lf_gain
        inputs[1] *= self.rf_gain
        inputs[2] *= self.lp_gain
        inputs[3] *= self.rp_gain
        inputs[4] *= self.e_gain

        # store new inputs
        self.inputs_hist.append(inputs)

        # use CTRNN to produce motor commands
        # here, I am using a tanh transfer function in the CTRNN,????
        # so the motor commands will be in the interval [-1, 1]
        commands = self.net.advance(inputs, self.time_constant, self.time_constant)

        # add noise to commands
        # if self.noisemakers_inds:
        #     for ind in self.noisemakers_inds:
        #         # check that there is a noisemaker and a command at the given index in each list
        #         if ind < len(self.noisemakers) and ind < len(commands):
        #             if self.noisemakers[ind]: # THIS IS REDUNDANT????????????
        #                 commands[ind] += self.noisemakers[ind].step(dt)

        # store new commands
        self.commands_hist.append(commands)

        # return commands
        return commands


'''

    A fitness function which evaluates how well the controller
    outputs appropriate commands, given the CTRNNs which are
    being evolved.

    Note that things are not normally this simple - this function is
    based on prior knowledge of what the controller should do, 
    -- the robot should seek the food and avoid the poison.

'''
def fit_func1(data):
    # get controller outputs
    # print('data:',data)
    commands_hist = data["commands_hist"]

    # convert controller outputs to numpy array, so that we can
    # extract columns
    c2 = np.array(commands_hist)
    m1 = np.mean(c2[:,0]) # mean output to left motor
    m2 = np.mean(c2[:,1]) # mean output to right motor

    # get distance from actual mean values to target mean values
    dist = np.linalg.norm([m1-0.22, m2-0.15])

    # convert distance to normalised fitness
    return 1/(1+dist)

#     fit = f_dist_m / (f_dist_m + p_dist_m) * e_m/200
def fit_func_distance(data, f_stim, p_stim):
    # robot coordinates
    xs = data["xs"]
    ys = data['ys']

    # food coordinates in whole duration
    f_xs_all=[]
    f_ys_all=[]
    for f in f_stim:
        f_xs_all.append(f.xs)
        f_ys_all.append(f.ys)

    # poison coordinates in whole duration
    p_xs_all=[]
    p_ys_all=[]
    for p in p_stim:
        p_xs_all.append(p.xs)
        p_ys_all.append(p.ys)

    # get distance from foods to robot
    f_dists=[]
    for f_xs, f_ys in zip(f_xs_all,f_ys_all):
        for f_x, f_y, x, y in zip(f_xs, f_ys, xs, ys):
            dist = math.sqrt( (f_x-x)**2 + (f_y-y)**2 )
            f_dists.append(dist)
    f_dist_m = np.mean(f_dists)

    # get distance from poisons to robot
    p_dists=[]
    for p_xs, p_ys in zip(p_xs_all, p_ys_all):
        for p_x, p_y, x, y in zip(p_xs, p_ys, xs, ys):
            dist = math.sqrt( (p_x-x)**2 + (p_y-y)**2 )
            p_dists.append(dist)
    p_dist_m = np.mean(p_dists)

    # convert mean of food poison distance and energy to normalised fitness
    fit = p_dist_m / (f_dist_m + p_dist_m)
    return fit

#     fit = e_m/200   robot don't move
def fit_func_energy(data):
    # mean robot energy
    e = data['energy']
    e_m = np.mean(e)

    # convert energy to normalised fitness  (energy / maxenergy)
    fit = e_m/200
    return fit

def fit_func_eat(data):
    # the number of times robot eat food / poison
    f_eat = data['f_eat']
    p_eat = data['p_eat']

    # the maximum number of times robot eat food / poison under ideal conditions
    # f_max_eat = food_num * (duration / recovery_time)
    # p_max_eat = poison_num * (duration / rcovery_time)
    f_max_eat = 15 * (300/50)
    p_max_eat = 10 * (300/50)

    # convert food and poison eating times to normalised fitness
    fit = (f_eat - p_eat) / (f_max_eat + p_max_eat)
    # fit = f_eat - p_eat
    return fit

def fit_func_combo1(data, f_stim, p_stim):
    # return (fit_func_distance(data,f_stim,p_stim) + fit_func_eat(data) + fit_func_energy(data))/3
    return 2/3 *fit_func_eat(data) + 1/3 * fit_func_energy(data)
    # return 4/5 *fit_func_eat(data) + 1/5 * fit_func_energy(data)

# fitness = ((np.mean(agent.food_consumed) - np.mean(agent.poison_consumed)) * agent.empty_steps/env.max_steps) + agent.hp/100



'''
    This function is used by NEAT to evaluate a single genotype (genome),
    using "config", which uses the parameters set in the "config-ctrnn"
    file.
'''
def eval_genome(genome, config):

    # controller parameters
    time_constant = 0.1
    gain = 2

    # if you want to use controller noise, set up noisemakers here

    controller = CTRNN_Controller(inputs_n=5, commands_n=2, gain=gain, genome=genome, config=config, time_constant=time_constant, noisemakers=None, noisemakers_inds=None)


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


    # energy_sensor_noisemaker = WhiteNoiseSource(min_val=-5, max_val=5)
    energy_sensor_noisemaker = None
    left_food_noisemaker = None  # WhiteNoiseSource(min_val=-0.5, max_val=0.5)
    right_food_noisemaker = None
    left_poison_noisemaker = None
    right_poison_noisemaker = None

    #########################################################################
    #                        Prepare simulation                             #
    #########################################################################

    food_num = 15
    poison_num = 10
    scale = 10
    recovery_time = 50

    foods_and_poisons = []
    foods_stimuli = []  # list of LightSources attached to food consumables, used by the robot's food sensors
    poisons_stimuli = []  # list of LightSources attached to food consumables, used by the robot's poison sensors
    # generate food items
    for i in range(food_num):
        x = random_in_interval(-scale, scale)
        y = random_in_interval(-scale, scale)
        food = Consumable(x, y, radius=1, recovery_time=recovery_time, quantity=25)
        foods_and_poisons.append(food)
        foods_stimuli.append(food.stimulus)
    # generate poison items
    for i in range(poison_num):
        x = random_in_interval(-scale, scale)
        y = random_in_interval(-scale, scale)
        poison = Consumable(x, y, radius=1, recovery_time=recovery_time, quantity=25, type=Consumables.poison)
        foods_and_poisons.append(poison)
        poisons_stimuli.append(poison.stimulus)

    # create robot
    robot = HungryRobot(x=0, y=0, controller=controller,
                        left_food_sources=foods_stimuli,  # the lights attached to food items
                        right_food_sources=foods_stimuli,
                        left_poison_sources=poisons_stimuli,  # the lights attached to poison items
                        right_poison_sources=poisons_stimuli,
                        left_food_sensor_angle=np.pi / 3,
                        # i only put a small difference between sensor pair angles so that we can see them all
                        right_food_sensor_angle=-np.pi / 3,
                        left_poison_sensor_angle=np.pi / 5,
                        right_poison_sensor_angle=-np.pi / 5,
                        food_field_of_view=0.8 * np.pi,  # sensor angular fields of view
                        poison_field_of_view=0.8 * np.pi,
                        consumables=foods_and_poisons,  # all of the consumables
                        energy_sensor_noisemaker=energy_sensor_noisemaker,
                        left_food_noisemaker=left_food_noisemaker,
                        right_food_noisemaker=right_food_noisemaker,
                        left_poison_noisemaker=left_poison_noisemaker,
                        right_poison_noisemaker=right_poison_noisemaker,
                        decay_rate=2,  # determines the rate at which motion costs energy
                        decay_rate2=0.01)  # some energy is lost even if the robot does not move

    # #########################################################################
    # #                      Simulation parameters                            #
    # #########################################################################

    # simulation run parameters
    screen_width = 780  # height and width of animation window, in pixels
    duration = 300  # number of simulation time units to simulate for
    n_runs = 1  # number of simulations to run
    animate = True  # whether or not to animate
    animation_frame_delay = 1  # the 10s of milliseconds to pause between frames
    #
    #########################################################################
    #                       Simulation section                              #
    #########################################################################

    # put robot into list of agents for Simulator
    agents = [robot]

    """A perturb function for perturbing the robot's pose."""

    # add randomness to robot's initial locations & orientations
    def perturb(self):
        x = self.x + random_in_interval(minimum=-10, maximum=10)
        y = self.y + random_in_interval(minimum=-10, maximum=10)
        theta = self.theta + random_in_interval(minimum=-math.pi, maximum=math.pi)
        # print(x, y, theta)
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
    sim = Simulator(agents=agents, envs=foods_and_poisons + foods_stimuli + poisons_stimuli, duration=duration, dt=0.1,
                    disturbances=disturbances)
    # get SimulationRunner object
    sim_runner = SimulationRunner(sim, animate=animate_, pause_ani=True, animation_delay=animation_frame_delay,
                                  screen_width=screen_width)
    # Run simulation n times
    sim_data = sim_runner.run_sims(n=n_runs)

    ########################################################################
    #                     Plotting and analysis section                     #
    #########################################################################

    if do_plots_:

        # plot agents' trajectories
        ax = plot_all_sims_trajectories(sim_data, show_cbar=True, cbar_fig=False)
        for light in foods_stimuli + poisons_stimuli:
            light.draw(ax)
        # plot robots' basic data (plots of robot's various noisemakers are not produced by this function call)
        plot_all_robots_basic_data(sim_data, multiple_plots=False, show_motors=True, show_controllers=True,
                                   show_sensors=True)
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

        print('Energy:', robot_data["energy_sensor"]["activations"][-1])
        print('Survival time steps:', robot_data['survival_time'], 'over', duration/0.1)
        print('Eat food:', robot_data['f_eat'])
        print('Eat poison', robot_data['p_eat'])
        # evalution indicator = 1/4 * survival_time/duration_time + 1/4 * energy/max_energy + 1/2* (food eat - poison eat)/ (max food eat +max poison eat)
        eval_indicator = 1 / 4 * robot_data['survival_time'] / 3000 \
                         + 1 / 4 * robot_data["energy_sensor"]["activations"][-1] / 200 \
                         + 1 / 2 * (robot_data['f_eat'] - robot_data['p_eat']) / (90 + 60)
        print('Adaptivity score:', eval_indicator)
        plt.show()

    # return fit_func1(sim_data[0]["agents"][0]["controller"])
    # return fit_func_distance(sim_data[0]["agents"][0],foods_stimuli,poisons_stimuli)
    # return fit_func_energy(sim_data[0]["agents"][0])
    return fit_func_eat(sim_data[0]['agents'][0])
    # return fit_func_combo1(sim_data[0]['agents'][0],foods_stimuli,poisons_stimuli)


def run_evolution():
    """run evolution"""
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config-ctrnn')
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    pop = neat.Population(config)
    stats = neat.StatisticsReporter()
    pop.add_reporter(stats)
    pop.add_reporter(neat.StdOutReporter(True))
    # pop.add_reporter(neat.Checkpointer(5))

    pe = neat.ParallelEvaluator(6, eval_genome)
    # print('pe:',pe)
    max_generation = 30  # run until solution is found, if not found run NEAT's genetic algorithm at most max_generation.
    winner = pop.run(pe.evaluate, max_generation)

    # Save the winner.
    with open('neat_results/winner-CTRNN', 'wb') as f:
        pickle.dump(winner, f)
    # print winner to terminal
    print('winner:',winner)

    # set global parameters to enable plots, and run simulation again for winner
    global do_plots_
    do_plots_ = True
    global animate_
    animate_ = True
    global runs_
    # run 20 times and plot
    runs_ = 1
    ee = eval_genome(winner, config)
    print('eval_genome fitness score:',ee)

    # plots to show how evolution progressed
    plt.figure()
    visualize.plot_stats(stats, ylog=True, view=True, filename="neat_results/CTRNN-fitness.svg")
    visualize.plot_species(stats, view=True, filename="neat_results/CTRNN-speciation.svg")

    ###### plots to show evolved network #####

    # it is possible to set the names of the nodes in the produced graph,
    # - but you have to work out which are which first!
    # node_names = {-1: '-1 - Right sensor', -2: '-2 - Left sensor', 0: '0 - left_speed', 1: '1 - right_speed'}
    node_names = {-1: '-1 - Energy sensor', -2: '-2 - Right poison sensor', -3: '-3 - Left poison sensor', -4: '-4 - Right food sensor', -5: '-5 - Left food sensor',
                  0: '0 - left_speed', 1: '1 - right_speed'}

    # these lines of code produce svg files of the network, and then those
    # svg files will be opened automatically, if view_svgs=True, and not if
    # view_svgs=False
    view_svgs = True
    # visualize.draw_net(config, winner, view=view_svgs, node_names=node_names)
    visualize.draw_net(config, winner, view=view_svgs, node_names=node_names,
                       filename="neat_results/winner-CTRNN.gv")
    visualize.draw_net(config, winner, view=view_svgs, node_names=node_names,
                       filename="neat_results/winner-CTRNN-enabled.gv", show_disabled=False)
    visualize.draw_net(config, winner, view=view_svgs, node_names=node_names,
                       filename="neat_results/winner-CTRNN-enabled-pruned.gv", show_disabled=False, prune_unused=True)


"""simulate the winner"""
def rerun():
    # p = neat.Checkpointer.restore_checkpoint('neat-checkpoint-4')
    # p.run(eval_genomes, 10)
    winner = pickle.load(open("neat_results_combo_0.4_23/winner-CTRNN", "rb"))   # the file path can be changed

    # Load the config file, which is assumed to live in
    # the same directory as this script.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config-ctrnn')
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    # set global parameters to enable animation and plotting, and run simulation once
    global animate_
    animate_ = True
    global do_plots_
    do_plots_ = True

    # run evaluation function once, so that we can see animation and plots
    eval_genome(winner, config)

    plt.show()

    # print winner to terminal
    # print(winner)

    ###### plots to show evolved network #####

    # it is possible to set the names of the nodes in the produced graph,
    # - but you have to work out which are which first!
    # node_names = {-1: '-1 - Right sensor', -2: '-2 - Left sensor', 0: '0 - left_speed', 1: '1 - right_speed'}
    node_names = {-1: '-1 - Energy sensor', -2: '-2 - Right poison sensor', -3: '-3 - Left poison sensor', -4: '-4 - Right food sensor', -5: '-5 - Left food sensor',
                  0: '0 - left_speed', 1: '1 - right_speed'}

    # these lines of code produce svg files of the network, and then those
    # svg files will be opened automatically, if view_svgs=True, and not if
    view_svgs=False
    # view_svgs = True
    # visualize.draw_net(config, winner, view=view_svgs, node_names=node_names)
    visualize.draw_net(config, winner, view=view_svgs, node_names=node_names,
                       filename="neat_results/winner-CTRNN.gv")
    visualize.draw_net(config, winner, view=view_svgs, node_names=node_names,
                       filename="neat_results/winner-CTRNN-enabled.gv", show_disabled=False)
    visualize.draw_net(config, winner, view=view_svgs, node_names=node_names,
                       filename="neat_results/winner-CTRNN-enabled-pruned.gv", show_disabled=False, prune_unused=True)



if __name__ == '__main__':
    # run_evolution()  # use this line to run evolution
    # rerun()  # use this line to simulate winner from a previous evolutionary run, winner has been saved to file in neat_result folder
    for i in range(5):
        rerun()