"""Generate a sine wave value with added Gaussian noise."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
import rospy
import sys
import os

# print(sys.path)
# here = os.path.abspath(os.path.dirname(__file__))
# there = os.path.normpath(os.path.join(here, '..'))
# sys.path.append(there)
# print(sys.path)
# os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
# # from yolomsg import Float32Stamped
from std_msgs.msg import Float64
import math
import random


def generateStochasticSineWave(frequency: float,
                               amplitude: float,
                               noiseMean: float,
                               noiseStdDev: float) -> float:
  """
  Generate a sine wave value with added Gaussian noise.

  Args:
    frequency: The frequency of the sine wave.
    amplitude: The amplitude of the sine wave.
    noiseMean: The mean of the Gaussian noise.
    noiseStdDev: The standard deviation of the Gaussian noise.

  Returns:
    A float representing the noisy sine wave value.
  """
  currentTime = rospy.get_time()
  sineValue = amplitude * math.sin(2 * math.pi * frequency * currentTime)
  noisySineValue = sineValue + random.gauss(noiseMean, noiseStdDev)
  return noisySineValue


def sampleWave():
  """
  Initialize ROS node and publish stochastic sine wave values.
  """
  rospy.init_node('LOL', anonymous=False)
  publisher = rospy.Publisher('/tool/pump_current', Float64,
                              queue_size=10)
  rate = rospy.Rate(10)  # 10 Hz

  while not rospy.is_shutdown():
    noisySineValue = generateStochasticSineWave(0.01, 1.0, 0.0, 0.1)
    rospy.loginfo(noisySineValue)
    # val = Float32Stamped(data=noisySineValue, )
    publisher.publish(noisySineValue)
    rate.sleep()


if __name__ == '__main__':
  try:
    sampleWave()
  except rospy.ROSInterruptException:
    pass
