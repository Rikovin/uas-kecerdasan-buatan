import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# MOHAMMAD KEVIN PUTRA ADIYAKSA
# 191021400022
# 06TPLE007

class FuzzyLogic:
    def __init__(self, input_distanceX, input_distanceY):
        self.input_distanceX = input_distanceX
        self.input_distanceY = input_distanceY

        self.distanceX = ctrl.Antecedent(np.arange(-150, 150, 1), 'distance X')
        self.speed = ctrl.Consequent(np.arange(-3, 9, 0.5), 'speed')

        self.distanceY = ctrl.Antecedent(np.arange(0, 140, 1), 'distance Y')
        self.turn = ctrl.Consequent(np.arange(-25, 25, 1), 'turn')

    def computeFuzzyResult(self):
        # distance y
        self.distanceY['stop'] = fuzz.trimf(self.distanceY.universe, [0, 0, 25])
        self.distanceY['slow'] = fuzz.trimf(self.distanceY.universe, [15, 30, 50])
        self.distanceY['normal'] = fuzz.trimf(self.distanceY.universe, [25, 70, 90])
        self.distanceY['fast'] = fuzz.trimf(self.distanceY.universe, [65, 85, 120])
        self.distanceY['veryFast'] = fuzz.trimf(self.distanceY.universe, [90, 140, 140])

        # speed
        self.speed['stop'] = fuzz.trimf(self.speed.universe, [-3, -3, 0])
        self.speed['slow'] = fuzz.trimf(self.speed.universe, [-0.5, 1.5, 3])
        self.speed['normal'] = fuzz.trimf(self.speed.universe, [1.5, 4, 4.5])
        self.speed['fast'] = fuzz.trimf(self.speed.universe, [3.5, 5, 8])
        self.speed['veryFast'] = fuzz.trimf(self.speed.universe, [6, 9, 9])

        # distance x
        self.distanceX['veryLeft'] = fuzz.trimf(self.distanceX.universe, [-160, -160, -40])
        self.distanceX['left'] = fuzz.trimf(self.distanceX.universe, [-140, -30, 0])
        self.distanceX['center'] = fuzz.trimf(self.distanceX.universe, [-30, 0, 30])
        self.distanceX['right'] = fuzz.trimf(self.distanceX.universe, [0, 30, 140])
        self.distanceX['veryRight'] = fuzz.trimf(self.distanceX.universe, [40, 160, 160])

        # turn
        self.turn['veryLeft'] = fuzz.trimf(self.turn.universe, [-25, -25, -8])
        self.turn['left'] = fuzz.trimf(self.turn.universe, [-18, -9, 0])
        self.turn['center'] = fuzz.trimf(self.turn.universe, [-1, 0, 1])
        self.turn['right'] = fuzz.trimf(self.turn.universe, [0, 9, 18])
        self.turn['veryRight'] = fuzz.trimf(self.turn.universe, [8, 25, 25])

        # View all the membership function
        # self.distanceY.view()
        # self.distanceX.view()
        # self.speed.view()
        # self.turn.view()

        # Fuzzy Implication Rule
        rule1 = ctrl.Rule(self.distanceY['stop'], self.speed['stop'])
        rule2 = ctrl.Rule(self.distanceY['slow'], self.speed['slow'])
        rule3 = ctrl.Rule(self.distanceY['normal'], self.speed['normal'])
        rule4 = ctrl.Rule(self.distanceY['fast'], self.speed['fast'])
        rule5 = ctrl.Rule(self.distanceY['veryFast'], self.speed['veryFast'])

        rule6 = ctrl.Rule(self.distanceX['veryLeft'], self.turn['veryLeft'])
        rule7 = ctrl.Rule(self.distanceX['left'], self.turn['left'])
        rule8 = ctrl.Rule(self.distanceX['center'], self.turn['center'])
        rule9 = ctrl.Rule(self.distanceX['right'], self.turn['right'])
        rule10 = ctrl.Rule(self.distanceX['veryRight'], self.turn['veryRight'])

        speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        turn_ctrl = ctrl.ControlSystem([rule6, rule7, rule8, rule9, rule10])

        robot_speed = ctrl.ControlSystemSimulation(speed_ctrl)
        head_turn = ctrl.ControlSystemSimulation(turn_ctrl)

        robot_speed.input['distance Y'] = self.input_distanceY
        robot_speed.compute()

        head_turn.input['distance X'] = self.input_distanceX
        head_turn.compute()

        walking_speed = robot_speed.output['speed']
        self.speed.view(sim=robot_speed)

        pan_turn = head_turn.output['turn']
        self.turn.view(sim=head_turn)

        return walking_speed, pan_turn


a = FuzzyLogic(input_distanceX=2, input_distanceY=1)
walking_speed, pan_turn = a.computeFuzzyResult()
plt.show()
print("Walking Speed: ", walking_speed)
print("Head Pan Turn: ", pan_turn)