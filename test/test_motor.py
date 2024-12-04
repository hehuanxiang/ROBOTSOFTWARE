from Stepper import Stepper
#from motor import Stepper

# testStepper = Stepper([13,15,11 ,40,37,38])#140

testStepper = Stepper([29,15,11 ,16,18,37])#140

#pull,dir,ena, end,rest,mag
#testStepper.step(40000, "right", 1, docking = False)

testStepper.step(500000, "right", 100, docking = True)      # right means moving forward



from Stepper import Stepper

# testStepper = Stepper([13,15,11 ,40,37,38])#140
# testStepper.step(3000000, "left", 5, docking = True)
