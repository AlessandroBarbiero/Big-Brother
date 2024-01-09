import csv
import numpy as np
# from lapjv import lapjv   # lapjv from lapjv can work only on square matrices
# from scipy.optimize import linear_sum_assignment   # works differently respect to C implementation, returns row and col instead of rowsol and colsol
from lap import lapjv # same used in C
from copy import deepcopy
from typing import Tuple
from collections import defaultdict
import math

class ObjectList:
    '''
    Class that represents a list of objects at a given timestamp for a given detection number.
    Different detections can have the same timestamp but different detection number
    '''
    def __init__(self, timestamp, detection_number, object_list):
        self.timestamp = timestamp
        self.detection_number = detection_number
        self.object_list = deepcopy(object_list)

    def __str__(self):
        return (f"Object list at {self.timestamp}, detection number: {self.detection_number}\n{self.object_list}")
    def __repr__(self):
        return (f"Object list at {self.timestamp}, detection number: {self.detection_number}\n{self.object_list}")

class Object3D:
    '''
    Class that represents a detected object with all the related characteristics
    '''
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
        self.minmax = [x-size_x/2.0, y-size_y/2.0, z-size_z/2.0, x+size_x/2.0, y+size_y/2.0, z+size_z/2.0]

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

def calculate_iou_cost(minmaxA : list, minmaxB : list) -> float:
    '''
    Computes the iou cost (1-iou) for the two given 3D axis-aligned bounding boxes represented by the min and max point of the box
    '''
    # Calculate the intersection coordinates
    inter_xmin = max(minmaxA[0], minmaxB[0])
    inter_ymin = max(minmaxA[1], minmaxB[1])
    inter_zmin = max(minmaxA[2], minmaxB[2])
    inter_xmax = min(minmaxA[3], minmaxB[3])
    inter_ymax = min(minmaxA[4], minmaxB[4])
    inter_zmax = min(minmaxA[5], minmaxB[5])

    # Calculate the intersection volume
    inter_volume = max(0, inter_xmax - inter_xmin + 1) * max(0, inter_ymax - inter_ymin + 1) * max(0, inter_zmax - inter_zmin + 1)

    # Calculate the union volume
    boxA_volume = (minmaxA[3] - minmaxA[0] + 1) * (minmaxA[4] - minmaxA[1] + 1) * (minmaxA[5] - minmaxA[2] + 1)
    boxB_volume = (minmaxB[3] - minmaxB[0] + 1) * (minmaxB[4] - minmaxB[1] + 1) * (minmaxB[5] - minmaxB[2] + 1)
    union_volume = boxA_volume + boxB_volume - inter_volume

    # Calculate IoU
    iou = inter_volume / union_volume if union_volume > 0 else 0.0
    return 1 - iou  # Subtract from 1 to convert IoU to a cost




def linear_assignment_obj(objects1: list, objects2: list, iou_thresh: float) -> Tuple[list, int, int, float, float]:
    """
    Given two lists of objects3D compute the cost matrix and the linear assignment using as cost functon the iou between two objects.
    A match is considered only if the iou is above the thresh
    :returns
        - list of matched pair indices
        - unmatched objects of the first list
        - unmatched objects of the second list
        - sum of cost (1-iou) for all matched pairs
        - sum of iou for all matched pairs
    """
    # Compute cost matrix based on IoU
    num_boxes1 = len(objects1)
    num_boxes2 = len(objects2)
    cost_matrix = np.zeros((num_boxes1, num_boxes2))

    for i in range(num_boxes1):
        for j in range(num_boxes2):
            cost_matrix[i, j] = calculate_iou_cost(objects1[i].minmax, objects2[j].minmax)

    # Solve the assignment problem using LAPJV algorithm
    return linear_assignment_cost_mat(cost_matrix=cost_matrix, iou_thresh=iou_thresh)

def linear_assignment_cost_mat(cost_matrix, iou_thresh):
    """
    Given a cost matrix compute the linear assignment.
    A match is considered only if the iou is above the thresh
    :returns
        - list of matched pair indices
        - unmatched objects of the first list
        - unmatched objects of the second list
        - sum of cost (1-iou) for all matched pairs
        - sum of iou for all matched pairs
    """
    thresh = iou_thresh
    tot_cost, x, y = lapjv(cost_matrix, extend_cost=True, cost_limit=thresh)
    matches = np.asarray([[ix, mx] for ix, mx in enumerate(x) if mx >= 0])

    zz = [cost_matrix[i[0], i[1]] for i in matches]
    tot_cost = sum(zz)
    tot_iou = sum(1 - val for val in zz)
    if len(matches) == 0:
        unmatched_a = list(np.arange(cost_matrix.shape[0]))
        unmatched_b = list(np.arange(cost_matrix.shape[1]))
    else:
        unmatched_a = list(set(np.arange(cost_matrix.shape[0])) - set(matches[:, 0]))
        unmatched_b = list(set(np.arange(cost_matrix.shape[1])) - set(matches[:, 1]))

    return matches, unmatched_a, unmatched_b, tot_cost, tot_iou


def read_csv(file_path : str) -> list:
    '''
    Reads the given csv file to extract a list of detections expressed as ObjectList, 
    each element of the returned list is a different detection and should have a different detection number
    '''
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

def compute_HOTA(track_history : list, gt_history : list, alpha : float):
    '''
    Returns the HOTA performance value using the given alpha
    :param track_history: a list of ObjectList elements representing the history of the tracking detections, 
            each element of the list is a different detection and should have a different detection number
    :param gt_history: a list of ObjectList elements representing the history of the ground truth registered at different detection times, 
            each element of the list is relative to a different detection and should have a different detection number
    :param alpha: the alpha parameter to compute the HOTA, it represents the IoU threshold in the computation of the Hungarian Algorithm
    :returns the HOTA_alpha value
    '''

    tot_true_positive       = 0
    tot_false_positive      = 0
    tot_missed              = 0

    # Variables to keep assA info
    gt_history_track = defaultdict(list)
    track_history_track = defaultdict(list)
    tp_track_history = []
    tp_gt_history = []
    for i in range(len(track_history)):
        track = track_history[i].object_list
        gt = gt_history[i].object_list
        pairs, unmatch_track, unmatch_gt, tot_cost, tot_iou = linear_assignment_obj(track, gt, 1-alpha)
        true_positive = len(pairs)
        false_positive = len(unmatch_track)
        missed = len(unmatch_gt)
        match_track_ids = [track[pair[0]].id for pair in pairs]
        match_gt_ids = [gt[pair[1]].id for pair in pairs]

        # computations for the assA, build two dictionaries that link each id to the history of linked objects of the other category
        for i in range(len(match_gt_ids)):
            gt_history_track[match_gt_ids[i]].append(match_track_ids[i])
            track_history_track[match_track_ids[i]].append(match_gt_ids[i])
        for gt_i in unmatch_gt:
            gt_history_track[gt[gt_i].id].append(None)
        for t_i in unmatch_track:
            track_history_track[track[t_i].id].append(None)

        tp_track_history.append(match_track_ids)
        tp_gt_history.append(match_gt_ids)

        tot_true_positive          +=  true_positive
        tot_false_positive         +=  false_positive
        tot_missed                 +=  missed
    
    # Compute assA
    tot_ass_iou = 0
    for match_track_ids, match_gt_ids in zip(tp_track_history, tp_gt_history):
        for track_id, gt_id in zip(match_track_ids, match_gt_ids):
            TPA = track_history_track[track_id].count(gt_id)
            FPA = len(track_history_track[track_id]) - TPA
            FNA = len(gt_history_track[gt_id]) - TPA
            ass_iou = TPA / (TPA + FPA + FNA)
            tot_ass_iou += ass_iou

    if(tot_true_positive != 0):
        detA = tot_true_positive / (tot_true_positive + tot_false_positive + tot_missed)
        assA = tot_ass_iou / tot_true_positive
    else:
        print(f"NO True positives for alpha = {alpha}!")
        detA = 0
        assA = 0

    return math.sqrt(detA * assA)


def main():

    print("Reading csv...")
    gt_history = read_csv('/home/ale/big_brother/data/gt.csv')
    track_history = read_csv('/home/ale/big_brother/data/track.csv')


    gt_index_to_track_id = {}
    tot_ass_mismatch        = 0
    tot_iou_matched         = 0.0
    tot_iou_cost_matched    = 0.0
    tot_true_positive       = 0
    tot_false_positive      = 0
    tot_missed              = 0
    tot_objects_to_detect   = 0

    print("Analyzing data...")
    # Variables to keep assA info
    gt_history_track = defaultdict(list)
    track_history_track = defaultdict(list)
    tp_track_history = []
    tp_gt_history = []
    for i in range(len(track_history)):
        track = track_history[i].object_list
        gt = gt_history[i].object_list
        pairs, unmatch_track, unmatch_gt, tot_cost, tot_iou = linear_assignment_obj(track, gt, 0.7)
        true_positive = len(pairs)
        false_positive = len(unmatch_track)
        missed = len(unmatch_gt)
        match_track_ids = [track[pair[0]].id for pair in pairs]
        match_gt_ids = [gt[pair[1]].id for pair in pairs]

        # computations for the assA, build two dictionaries that link each id to the history of linked objects of the other category
        for i in range(len(match_gt_ids)):
            gt_history_track[match_gt_ids[i]].append(match_track_ids[i])
            track_history_track[match_track_ids[i]].append(match_gt_ids[i])
        for gt_i in unmatch_gt:
            gt_history_track[gt[gt_i].id].append(None)
        for t_i in unmatch_track:
            track_history_track[track[t_i].id].append(None)

        tp_track_history.append(match_track_ids)
        tp_gt_history.append(match_gt_ids)

        # Compute association mismatch
        for i in range(len(match_gt_ids)):
            gt_index = match_gt_ids[i]
            track_id = match_track_ids[i]
            
            if gt_index in gt_index_to_track_id and gt_index_to_track_id[gt_index] != track_id:
                tot_ass_mismatch += 1
            
            gt_index_to_track_id[gt_index] = track_id

        tot_iou_matched         +=  tot_iou
        tot_iou_cost_matched    +=  tot_cost

        tot_true_positive          +=  true_positive
        tot_false_positive         +=  false_positive
        tot_missed                 +=  missed
        tot_objects_to_detect      +=  len(gt)

    
    # Compute assA
    tot_ass_iou = 0
    tot_TPA = 0
    tot_FPA = 0
    tot_FNA = 0
    for match_track_ids, match_gt_ids in zip(tp_track_history, tp_gt_history):
        for track_id, gt_id in zip(match_track_ids, match_gt_ids):
            TPA = track_history_track[track_id].count(gt_id)
            FPA = len(track_history_track[track_id]) - TPA
            FNA = len(gt_history_track[gt_id]) - TPA
            ass_iou = TPA / (TPA + FPA + FNA)
            tot_ass_iou += ass_iou
            tot_TPA += TPA
            tot_FPA += FPA
            tot_FNA += FNA

    if(tot_true_positive != 0):
        detA = tot_true_positive / (tot_true_positive + tot_false_positive + tot_missed)
        locA = tot_iou_matched / tot_true_positive
        assA = tot_ass_iou / tot_true_positive
        MOTP = tot_iou_cost_matched / tot_true_positive
    
    
    MOTA = 1.0 - (tot_missed + tot_false_positive + tot_ass_mismatch)/tot_objects_to_detect
    detRe = tot_true_positive / (tot_true_positive + tot_missed)
    detPr = tot_true_positive / (tot_true_positive + tot_false_positive)
    detF1 = 2.0 * (detPr * detRe)/(detPr+detRe)
    assRe = tot_TPA / (tot_TPA + tot_FNA)
    assPr = tot_TPA / (tot_TPA + tot_FPA)
    assF1 = 2.0 * (assPr * assRe)/(assPr+assRe)


    print(f"\
    ############## STATS ################\n\
    true positive:\t{tot_true_positive}\n\
    false positive:\t{tot_false_positive}\n\
    missed:\t\t{tot_missed}\n\
    ass. mismatch:\t{tot_ass_mismatch}\n\
    total objects:\t{tot_objects_to_detect}\n\
    \n\
    detA:\t{detA:.4f}\n\
    locA:\t{locA:.4f}\n\
    assA:\t{assA:.4f}\n\
    \n\
    DetRe:\t{detRe:.4f}\n\
    DetPr:\t{detPr:.4f}\n\
    DetF1:\t{detF1:.4f}\n\
    AssRe:\t{assRe:.4f}\n\
    AssPr:\t{assPr:.4f}\n\
    AssF1:\t{assF1:.4f}\n\
    \n\
    MOTP:\t{MOTP:.4f}\n\
    MOTA:\t{MOTA:.4f}\n\
    #####################################" \
    )

    print("Computing HOTA...")
    tot_HOTA_alpha = 0
    for alpha in np.arange(0.05, 1, 0.05):
        HOTA_alpha = compute_HOTA(track_history, gt_history, alpha)
        print(f"alpha = {alpha:.2f} -> HOTA_a = {HOTA_alpha:.4f}")
        tot_HOTA_alpha += HOTA_alpha

    HOTA = tot_HOTA_alpha / 19.0
    print(f"\n\
    HOTA:\t{HOTA:.4f}\n" \
    )





        
if __name__ == '__main__':
    main()