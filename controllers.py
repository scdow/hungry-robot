import sys
# relative path to folder which contains the Sandbox module
sys.path.insert(1, '../../')
from Sandbox_V1_3 import *

# a simple controller for a HungryRobot. As simple as this is, it will tend to drive towards food and away from poison.
# it doesn't work perfectly - I have noticed that it will sometimes drive over a poison object if there is food directly
# behind it
class HungryController(Controller):

    # construct controller
    def __init__(self, step_fun: Callable[[float, List[float], List[float], List[float]], List[float]], left_noisemaker: NoiseSource=None, right_noisemaker: NoiseSource=None, gain=1):

        noisemakers = []
        noisemakers_inds = []
        if left_noisemaker:
            noisemakers.append(left_noisemaker)
            noisemakers_inds.append(0)

        if right_noisemaker:
            noisemakers.append(right_noisemaker)
            noisemakers_inds.append(1)

        super().__init__(inputs_n=5, commands_n=2, step_fun=step_fun, noisemakers=noisemakers, noisemakers_inds=noisemakers_inds, params=[gain])

  # inputs[0] = left_food_activation
  # inputs[1] = right_food_activation
  # inputs[2] = left_poison_activation
  # inputs[3] = right_poison_activation
  # inputs[4] = energy_activation  # - not used in this basic controller, but a really good one certainly would pay attention to it!
def hungry_stepfun(dt: float, inputs: List[float], params: List[float], state: List[float]=[]) -> List[float]:

    left_speed_command = params[0]*(inputs[1]+2*inputs[2])
    right_speed_command =  params[0]*(inputs[0]+0.05)

    return [left_speed_command, right_speed_command]

def hungry_stepfun2 (dt: float, inputs: List[float], params: List[float], state: List[float]=[]) -> List[float]:
    # print(inputs)
    # print('energy cur:',inputs[-1])  # print current energy
    # [2, 0.1, 10]

    weight1, weight2, weight3 = params[0], params[0], 0.007
    # YELLOW LIGHT-SEEKEING
    # set left motor speed
    left_speed_command = weight1 * inputs[1]
    # set right motor speed
    right_speed_command = weight1 * inputs[0]

    # RED LIGHT AVOIDING
    # add to left motor speed
    left_speed_command += weight2 * inputs[2]
    # add to right motor speed
    right_speed_command += weight2 * inputs[3]

    # BIAS TO TURN SLOWLY WHEN NO LIGHTS DETECTED
    left_speed_command += 0.3
    right_speed_command += weight3 * inputs[4]  # consider the effects of energy

    return [left_speed_command, right_speed_command]
#
# def hungry_stepfun3(dt: float, inputs: List[float], params: List[float], state: List[float] = []) -> List[float]:
#     # print(inputs)
#     print(inputs[-1])  # print current energy
#     # [2, 0.1, 10]
#     weight1, weight2, weight
#     gain_l, gain_r = 0, 0
#
#
#     # YELLOW LIGHT-SEEKEING
#     # set left motor speed
#     left_speed_command = params[0] * inputs[1]
#     # set right motor speed
#     right_speed_command = params[0] * inputs[0]
#
#     # RED LIGHT AVOIDING
#     # add to left motor speed
#     left_speed_command += params[0] * inputs[2]
#     # add to right motor speed
#     right_speed_command += params[0] * inputs[3]
#
#     # BIAS TO TURN SLOWLY WHEN NO LIGHTS DETECTED
#     left_speed_command += 0.3
#
#     return [left_speed_command, right_speed_command]
#
#     # controller = CTRNN_Controller(inputs_n=2, commands_n=2, gain=gain, genome=genome, config=config, time_constant=time_constant, noisemakers=None, noisemakers_inds=None)

