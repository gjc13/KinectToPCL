__author__ = 'gjc13'

from numpy import array
from numpy import polyfit
import numpy
import struct
import matplotlib.pyplot as pyplot
from scipy.ndimage.filters import gaussian_filter1d
from numpy.linalg import eig
from mpl_toolkits.mplot3d import Axes3D
from scipy import misc
import sys

grid_x_length = 6.7
grid_y_length = 5.65

xstarts = [208, 221, 229, 232, 235, 237, 241, 240]
xends = [374, 347, 328, 316, 309, 301, 297, 293]
ystarts = [129, 150, 162, 172, 178, 184, 186, 190]
yends = [232, 229, 230, 228, 226, 227, 226, 226]


def read_depth_image(depth_filename):
    """

    :param depth_filename: string
    :return: (numpy.array, int, int
    """
    f = open(depth_filename, "r")
    byte_buffer = f.read(8)
    size = struct.unpack('ii', byte_buffer)
    arr = numpy.zeros(size)
    for i in range(0, size[0]):
        for j in range(0, size[0]):
            byte_buffer = f.read(4)
            arr[i, j] = struct.unpack('f', byte_buffer)[0]
    return arr, size[0], size[1]


def get_depth(arr, xfrom, xto, yfrom, yto):
    vals = array([arr[x, y] for x in range(xfrom, xto) for y in range(yfrom, yto) if arr[x, y] > 0.05])
    print "%d points" % len(vals)
    print "mean: %f" % vals.mean()
    print "var %f" % vals.var()
    print "min %f max %f" % (vals.min(), vals.max())


def get_min_points(input_list):
    """

    :param input_list: numpy.array
    :return: numpy.array
    """
    threshold = input_list.mean()
    possible_min_points = [i for i in range(1, len(input_list) - 1) if
                           input_list[i] < threshold and
                           input_list[i - 1] > input_list[i] < input_list[i + 1]]
    is_consecutive = False
    min_points = []
    last_index = -2
    start_index = 0
    for index in possible_min_points:
        if index is last_index + 1:
            is_consecutive = True
        else:
            if is_consecutive:
                min_points.append((start_index + last_index) / 2)
            else:
                min_points.append(index)
            is_consecutive = False
            start_index = index
        last_index = index
    if is_consecutive:
        min_points.append((start_index + last_index) / 2)
    if len(min_points) == 6:
        min_points.append(len(input_list) - 1)
    return array(min_points)


def get_xy_min_points(registered_image, index):
    xstart = xstarts[index]
    xend = xends[index]
    ystart = ystarts[index]
    yend = yends[index]
    board_image = registered_image[ystart:yend, xstart:xend, 0]
    xval = board_image.sum(axis=0)
    yval = board_image.sum(axis=1)
    xval = gaussian_filter1d(xval, sigma=1)
    yval = gaussian_filter1d(yval, sigma=1)
    x_min_points = get_min_points(xval)
    y_min_points = get_min_points(yval)
    return x_min_points, y_min_points


def build_calibrate_matrix_rows(calibrate_matrix_list, x_min_points, y_min_points, real_z, xstart, ystart):
    print(real_z)
    for j in range(0, len(x_min_points)):
        for k in range(0, len(y_min_points)):
            pixel_x = xstart + x_min_points[j]
            pixel_y = ystart + y_min_points[k]
            real_x = grid_x_length * (6 - j)
            real_y = grid_y_length * (4 - k)
            calibrate_matrix_list.append(
                [real_x, real_y, real_z, 1, 0, 0, 0, 0, -pixel_x * real_x, -pixel_x * real_y, -pixel_x * real_z,
                 -pixel_x]
            )
            calibrate_matrix_list.append(
                [0, 0, 0, 0, real_x, real_y, real_z, 1, -pixel_y * real_x, -pixel_y * real_y, -pixel_y * real_z,
                 -pixel_y]
            )


if __name__ == '__main__':
    # arr, num_rows, num_cols = read_depth_image("/Users/gjc13/KinectData/depth0.bin")
    # print num_rows, num_cols
    # xs = array([x for x in range(0, num_rows) for y in range(0, num_cols)])
    # ys = array([y for x in range(0, num_rows) for y in range(0, num_cols)])
    # zs = array([arr[x, y] for x in range(0, num_rows) for y in range(0, num_cols)])
    #
    # get_depth(arr, 211, 307, 153, 208)

    # distances = array([90, 120, 150, 180, 210, 240, 270, 300])
    # depth = array([0.210495, 0.277751, 0.343088, 0.411216, 0.479888, 0.546084, 0.613406, 0.680187])
    # p = polyfit(depth, distances, 1)
    # print p
    # pyplot.plot(distances, depth)
    # pyplot.show()
    calibrate_matrix_list = []

    registered_image = pyplot.imread('/Users/gjc13/KinectData/registered0.png')
    pyplot.imshow(registered_image)
    pyplot.show()
    print(registered_image[0][0])

    for i in range(0, 8):
        registered_image = pyplot.imread('/Users/gjc13/KinectData/registered' + str(i) + '.png')
        real_z = 30 * (i + 3)
        x_min_points, y_min_points = get_xy_min_points(registered_image, i)

        build_calibrate_matrix_rows(calibrate_matrix_list, x_min_points, y_min_points, real_z, xstarts[i], ystarts[i])
    calibrate_matrix = array(calibrate_matrix_list)
    calibrate_matrix_square = calibrate_matrix.T.dot(calibrate_matrix)
    eigenvalue, eigenvectors = eig(calibrate_matrix_square)
    print("eigenvalues:")
    print(eigenvalue)
    eigenvector = eigenvectors[:, len(eigenvectors) - 1]
    print("eigenvector:")
    print(eigenvector)
    print("ATA*eigenvector:")
    print(calibrate_matrix_square.dot(eigenvector) / eigenvector)
    projection_matrix = numpy.zeros((3, 4))
    for i in range(0, 3):
        for j in range(0, 4):
            projection_matrix[i, j] = eigenvector[4 * i + j]
    print("projection_matrix")
    print(projection_matrix)

    ws = []
    for depth in range(90, 330, 30):
        position_vec = array([0, 0, depth, 1])
        ws.append(projection_matrix.dot(position_vec)[2])
    ws = array(ws)
    depths = array(range(90, 330, 30))
    print(polyfit(depths, ws, 1))
    pyplot.plot(ws)
    pyplot.show()

    # pyplot.subplot(311)
    # pyplot.plot(xval)
    # pyplot.subplot(312)
    # pyplot.plot(yval)
    # pyplot.subplot(313)
    # pyplot.imshow(board_image)
    # pyplot.show()

    # fig = pyplot.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.view_init(elev=60, azim=75)
    # ax.scatter(zs, xs, ys)
    # pyplot.show()
