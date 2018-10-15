import os
import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from PIL import Image
import matplotlib

class Ecoli_obj(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.pointsx = []
        self.pointsy = []
        self.error = 0
        self.angles = []
        self.angle_diff = []
        self.history = []

    def set_pos_from_points(self, x_transform, y_transform):
        self.x = np.mean(self.pointsx) + x_transform
        self.y = np.mean(self.pointsy) + y_transform

    def calc_angle_diff(self):
        if (len(self.angles)==1):
            return 0
        else:
            no = self.angles[-2]+np.pi
            nx = self.angles[-1]+np.pi
            diff = nx-no
            if diff > np.pi:
                diff = diff - 2*np.pi
            elif diff < -np.pi:
                diff = diff + 2*np.pi
            return diff*180/np.pi
