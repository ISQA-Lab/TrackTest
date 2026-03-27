import os
import subprocess
import sys
def get_truncation(labels:KittiTrackingLabel,frame,id):
    labels = labels.get_labels_by_frame_id(frame)
    for label in labels:
        if label.object_id == id:
            return label.truncation

def truncation_score(trajectory_path,thresh,folder_path=None):
    truncation = []
    for file in os.listdir(folder_path):
        folder_path = os.path.join(folder_path,file)
    for file in os.listdir(folder_path):
        labels = KittiTrackingLabel(os.path.join(folder_path,file))
        frame = int(file.split(".")[0])
        labels = labels.get_labels_by_frame_id(frame)
        for label in labels:
            if label.cls_type not in ['Car','Van','Truck']:
                continue
            if label.truncation >= thresh:
                truncation.append(1)
            else:
                truncation.append(0)
    if len(truncation) == 0:
        return 0.0
    return sum(truncation)/len(truncation)
