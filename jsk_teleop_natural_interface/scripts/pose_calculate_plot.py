from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
# x = np.arange(-3, 3, 0.25)
# y = np.arange(-3, 3, 0.25)
# X, Y = np.meshgrid(x, y)
# Z = np.sin(X)+ np.cos(Y)

# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot_wireframe(X,Y,Z)

# plt.show()

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


    def plot(self):
        if(len(self.x) > 0 and len(self.y) > 0 and len(self.z) > 0
           and len(self.x) == len(self.y) == len(self.z)):
            self.ax.plot(self.x, self.y, self.z, label="Bone Line")
            if(self.spline_result != None):
                self.ax.plot(self.spline_result[0], self.spline_result[1], self.spline_result[2], label="Spline Result")
            plt.show()
        else:
            print "Invalid Value is set in x y z"



if __name__ == "__main__":
    test_bone_convert = BoneToCurve()
    test_bone_convert.set_pos(test_x, test_y, test_z)
    test_bone_convert.calc_interpolation()
    test_bone_convert.plot()
