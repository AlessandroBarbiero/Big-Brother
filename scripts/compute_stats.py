import csv
import numpy as np
from lapjv import lapjv
import scipy
from copy import deepcopy

#TODO: Check

class ObjectList:
    def __init__(self, timestamp, detection_number, object_list):
        self.timestamp = timestamp
        self.detection_number = detection_number
        self.object_list = deepcopy(object_list)

    def __str__(self):
        return (f"{self.timestamp}, number {self.detection_number}")


class Object3D:
    def __init__(self, timestamp : int, detection_number : int, obj_id : int, obj_class : str,
                  x : float, y : float, z : float, theta : float, size_x : float, size_y : float, size_z : float):
        self.timestamp = timestamp
        self.detection_number = detection_number
        self.id = obj_id
        self.obj_class = obj_class
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.minmax = [x-size_x/2, y-size_y/2, z-size_z/2, x+size_x/2, y+size_y/2, z+size_z/2]

    def __str__(self):
        return (f"Timestamp: {self.timestamp}, "
                f"Detection Number: {self.detection_number}, "
                f"ID: {self.id}, "
                f"Class: {self.obj_class}, "
                f"Coordinates (x, y, z): ({self.x}, {self.y}, {self.z}), "
                f"Theta: {self.theta}, "
                f"Sizes (X, Y, Z): ({self.size_x}, {self.size_y}, {self.size_z})")
    
    def __repr__(self):
        return (f"Timestamp: {self.timestamp}, "
                f"Detection Number: {self.detection_number}, "
                f"ID: {self.id}, "
                f"Class: {self.obj_class}, "
                f"Coordinates (x, y, z): ({self.x}, {self.y}, {self.z}), "
                f"Theta: {self.theta}, "
                f"Sizes (X, Y, Z): ({self.size_x}, {self.size_y}, {self.size_z})\n")

def calculate_iou_cost(boxA, boxB):
    # Calculate the intersection coordinates
    inter_xmin = max(boxA[0], boxB[0])
    inter_ymin = max(boxA[1], boxB[1])
    inter_zmin = max(boxA[2], boxB[2])
    inter_xmax = min(boxA[3], boxB[3])
    inter_ymax = min(boxA[4], boxB[4])
    inter_zmax = min(boxA[5], boxB[5])

    # Calculate the intersection area
    inter_area = max(0, inter_xmax - inter_xmin) * max(0, inter_ymax - inter_ymin) * max(0, inter_zmax - inter_zmin)

    # Calculate the union area
    boxA_area = (boxA[3] - boxA[0]) * (boxA[4] - boxA[1]) * (boxA[5] - boxA[2])
    boxB_area = (boxB[3] - boxB[0]) * (boxB[4] - boxB[1]) * (boxB[5] - boxB[2])
    union_area = boxA_area + boxB_area - inter_area

    # Calculate IoU
    iou = inter_area / union_area if union_area > 0 else 0
    return 1 - iou  # Subtract from 1 to convert IoU to a cost




def linear_assignment(objects1, objects2):
    # Calculate cost matrix based on IoU
    num_boxes1 = len(objects1)
    num_boxes2 = len(objects2)
    cost_matrix = np.zeros((num_boxes1, num_boxes2))

    for i in range(num_boxes1):
        for j in range(num_boxes2):
            cost_matrix[i, j] = calculate_iou_cost(objects1[i].minmax, objects2[j].minmax)

    # Solve the assignment problem using LAPJV algorithm
    row_ind, col_ind = scipy.optimize.linear_sum_assignment(cost_matrix)

    # Match the indices to get corresponding pairs of boxes
    matched_pairs = [(i, col_ind[i]) for i in range(min(num_boxes1, num_boxes2)) if col_ind[i] != -1]
    return matched_pairs


def read_csv(file_path):
    data = []

    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        
        # Skip the header row if it contains column names
        header = next(reader)
        
        for row in reader:
            new_row = [int(e) if i in [0,1,2] else str(e) if i==3 else float(e) for i, e in enumerate(row)]
            data.append(Object3D(*new_row))
        
    gts = []

    n = data[0].detection_number
    analize = []
    for row in data:
        if row.detection_number == n:
            analize.append(row)
        else:
            objlist = ObjectList(analize[0].timestamp, analize[0].detection_number, analize)
            gts.append(objlist)
            n = row.detection_number
            analize = [row]

    return gts

def main():

    gt_history = read_csv('/home/ale/big_brother/data/gt.csv')
    track_history = read_csv('/home/ale/big_brother/data/track.csv')

    for i in range(len(track_history)):
        pairs = linear_assignment(track_history[i].object_list, gt_history[i].object_list)
        print(pairs)




        
if __name__ == '__main__':
    main()