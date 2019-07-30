#   This script is ran from a root folder that contains multiple recordings (sets) in separate folders
#   It takes each frame from each sequence from each recording (set) and labels them appropriately so 
#   they can be in a folder used as a "datapath" attribute for a DataGenerator

import os
import shutil

def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isdir(os.path.join(a_dir, name))]
out_dir = 'images'
if not os.path.exists(out_dir):
    os.makedirs(out_dir)
sets = []
for folder in get_immediate_subdirectories(os.getcwd()):
    set_path = os.path.join(os.getcwd(), folder)
    sets.append(set_path)

for set_number, set_path in enumerate(sets):
    seq = []
    
    for sequence in get_immediate_subdirectories(set_path):
        sequence_path = os.path.join(set_path, sequence)
        seq.append(sequence_path)

    for sequence_number, sequence_folder in enumerate(seq):
        for frame_no, image in  enumerate(os.listdir(sequence_folder)):
            name = "set" + str(set_number).zfill(2) + '_V' + str(sequence_number).zfill(3) + '_' + str(frame_no) + ".png"
            currentPath = os.path.join(os.path.join(os.getcwd(), sequence_folder), image)
            newPath = os.path.join(os.path.join(os.getcwd(), out_dir), name)
            if image.endswith(".png"):
                shutil.move(currentPath, newPath)


