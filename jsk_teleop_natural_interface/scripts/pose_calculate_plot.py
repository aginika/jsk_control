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
        self.option_plot_dict_array={}
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

    def register_option_plot_line(self, label, points):
        self.option_plot_dict_array[label] = points

    def plot(self):
        if(len(self.x) > 0 and len(self.y) > 0 and len(self.z) > 0
           and len(self.x) == len(self.y) == len(self.z)):
            #Bone Line
            self.ax.plot(self.x, self.y, self.z, label="Bone Line")

            #draw spline_result
            if(self.spline_result != None):
                print "plot : Spline Result"
                self.ax.plot(self.spline_result[0], self.spline_result[1], self.spline_result[2], label="Spline Result")

            #draw option  line
            for label,points in self.option_plot_dict_array.items():
                print "plot : ", label
                self.ax.plot(points[0], points[1], points[2],label = label)
            self.ax.legend()
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


class SearchOptimizedFittedArm:
    def __init__(self,search_origin_pos,link_length_array):
        self.search_origin_pos = search_origin_pos
        self.link_length_array = link_length_array
        print "Init SearchOptimizedFitted Arm with"
        print "             search_origin_pos : ",self.search_origin_pos
        print "             link_length_array : ",self.link_length_array

    def solve(self, target_line, mode = 1):
        if mode == 1:
            return self.base_solver(target_line)
        else:
            print "Invalid Mode Setted!!!"

    def base_solver(self, target_line):
        tmp_search_origin_pos = np.array(self.search_origin_pos)
        result_points = [self.search_origin_pos]

        #we want the arm which is as far from other arm as possible
        vector_evaluate_bias = np.array([0,0,0])

        for num in range(len(self.link_length_array)):
            diff_vector = np.array(target_line[0]) - tmp_search_origin_pos
            euclid_dist_min = float("inf")

            dist_min_index = -1
            print "link_length_list : ", self.link_length_array[num]
            for i in range(len(target_line)):
                diff_vector = np.array(target_line[i]) - tmp_search_origin_pos
                diff_vector_norm = abs(np.linalg.norm(diff_vector, ord=1) - self.link_length_array[num])

                #add the bias which is calculated from vector inner product
                diff_vector_norm -= np.dot(diff_vector, vector_evaluate_bias)
                if diff_vector_norm < euclid_dist_min:
                    dist_min_index = i
                    euclid_dist_min = diff_vector_norm

            #set the next search origin_pos
            print "target_line[i] and i : ", target_line[dist_min_index], "    ",dist_min_index," minumum value = ",euclid_dist_min
            #            print "result_points : ",result_points
            result_points += [list(target_line[dist_min_index])]
            vector_evaluate_bias = target_line[dist_min_index] - tmp_search_origin_pos
            tmp_search_origin_pos = target_line[dist_min_index]
        return result_points

if __name__ == "__main__":
    test_bone_convert = BoneToCurve()
    test_bone_convert.set_pos(test_x, test_y, test_z)
    test_bone_convert.calc_interpolation()

    test_search = SearchOptimizedFittedArm([0,0,0], [3,4,3,2,3,4,2,4])
    get_approx_result = test_search.solve(test_bone_convert.get_spline_result().T)

    print "get_approx_result : ",np.array(get_approx_result).T
    test_bone_convert.register_option_plot_line("approx_result", np.array(get_approx_result).T)
    test_bone_convert.plot()
