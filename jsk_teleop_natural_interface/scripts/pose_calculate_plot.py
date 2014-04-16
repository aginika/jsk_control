#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
import random

#test Values
test_x = [0,1,3,7,5]
test_y = [0,2,5,3,9]
test_z = [0,3,6,5,3]

class BoneToCurve:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        plt.result_title = "Bone And Curve"
        self.spline_result = None

    def set_pos(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def calc_interpolation(self):
        points = np.array([self.x, self.y, self.z]);
        t_array = np.array(range(len(self.x)));

        self.spline_f = interpolate.interp1d(t_array, points, kind='cubic')
        self.spline_result = self.spline_f(np.linspace(0,len(self.x)-1,100))

    def get_spline_result(self):
        return self.spline_result

    def plot(self):
        if(len(self.x) > 0 and len(self.y) > 0 and len(self.z) > 0
           and len(self.x) == len(self.y) == len(self.z)):
            self.ax.plot(self.x, self.y, self.z, label="Bone Line")
            if(self.spline_result != None):
                self.ax.plot(self.spline_result[0], self.spline_result[1], self.spline_result[2], label="Spline Result")
            plt.show()
        else:
            print "Invalid Value is set in x y z"

class RandomArmGenerator:
    def __init__(self, arm_length,min_length = 2, max_length = 5):
        self.arm_length = arm_length
        self.min_length = min_length
        self.max_length = max_length
        print "Random Arm Init : arn_length = ",self.arm_length
        print "                  min_length = ",min_length
        print "                  max_length = ",max_length

    def generate_arm_length_array(self, link_num):
        depriving_length = self.arm_length - link_num * self.min_length
        print "depriving_length = ",depriving_length
        print "        link_num = ",link_num
        separate_result = self.split_array(depriving_length, link_num)
        generated_arm_length_array = map(lambda x: x+self.min_length, separate_result)
        print generated_arm_length_array
        random.shuffle(generated_arm_length_array)
        print generated_arm_length_array
        return generated_arm_length_array

    def split_array(self, all_length, link_num):
        arm_length_array = []
        for i in range(0, link_num):
            rest_length = all_length - sum(arm_length_array)
            if rest_length  > 0:
                new_link_length = random.randrange(0, self.max_length, 1)
                if new_link_length < rest_length:
                    arm_length_array += [new_link_length]
                else:
                    arm_length_array += [rest_length]
            else:
                arm_length_array += [0]
        return arm_length_array


if __name__ == "__main__":
    test_bone_convert = BoneToCurve()
    test_bone_convert.set_pos(test_x, test_y, test_z)
    test_bone_convert.calc_interpolation()
    test_bone_convert.plot()
