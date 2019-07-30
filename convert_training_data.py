# This script takes a raw recording in the same format as the _out folder in the carla/PythonAPI/examples directory
# and separates it in different sequences that are longer than MINIMUM_RANGE and move them in separate folders along
# with the respective data from the main CSV from the original recording

import sys
import csv
import numpy
from shutil import move
import os
import time
from PIL import Image, ImageDraw
import operator
import json

#This value represent the smallest acceptable continuous video sample (in frame count)
MINIMUM_RANGE = 30

def get_path_to_batch(batch_id):
    folder_path = os.getcwd()
    folder_path = folder_path + "/" + str(batch_id)
    return folder_path


def find_missing_frames():
    missing_frames = set()
    is_initial_frame = True
    total = 0
    current_frame = 0
    with open(sys.argv[1]) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if is_initial_frame:
                if int(row[0]) is not 0:
                    current_frame = int(row[0])
                    first_frame = row[0]
                    is_initial_frame = False
            else:
                if (int(row[0]) != current_frame) and (int(row[0]) != (current_frame + 1)):
                    # add all frames in between
                    for i in range(1, (int(row[0]) - current_frame )):
                        missing_frames.add(current_frame+i)
                current_frame = int(row[0])
    
    #add last frame
    missing_frames.add(current_frame+1)

    print(sorted(missing_frames))
    #TODO add last+1 
    return sorted(missing_frames), first_frame, total

def find_largest_batch(missing_frames_array, first_frame):
    begin = int(first_frame)
    long_ranges = {}
    
    #TODO: manage cases where there is only one frame missing
    if(len(missing_frames_array)) is 1:
        print("Case not managed")
        exit
    if(len(missing_frames_array)) is 0:
        long_ranges[str(begin)] = str(-1)
    else:
        if (missing_frames_array[0] - int(first_frame)) > MINIMUM_RANGE:
            long_ranges[first_frame] = str(missing_frames_array[0]-1)

        for i in range(1, len(missing_frames_array)):
            gap = (missing_frames_array[i]-1) - (missing_frames_array[i-1] + 1)
            if gap > MINIMUM_RANGE:
                begin = missing_frames_array[i-1] + 1
                long_ranges[str(begin)] = str(begin + gap)
    print(long_ranges)
    return long_ranges

def get_formatted_name(name):
    length = len(name)
    zeros = 8-length
    string = str()
    for x in range(zeros):
        string += str(0)
    string += name +".png"
    return string

def copy_to_folder(begin, end, batch_id, total):
    rows = []
    with open(sys.argv[1]) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if end is -1:
                rows.append(row)
            else: 
                if int(row[0]) >= begin and int(row[0]) <= end:
                    rows.append(row)
        csv_file.close()

    # create new folder 
    path = os.getcwd()
    source_path = path.join(str(batch_id))
    csv_name = str(batch_id) + ".csv"

    try:
        os.mkdir(source_path)
    except OSError:
        print("Creation of the directory %s failed" % source_path)
    else:
        print("Successfully created the directory %s " % source_path)

    csv_log_path = os.path.join(source_path, csv_name)
    with open(csv_log_path, 'w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(rows)
        csv_file.close()

    image_list = []
    for row in rows:
        image_list.append(get_formatted_name(row[0]))

    count = 0
    for filename in os.listdir(os.getcwd()):
        if filename.endswith(".png") and filename in image_list:
            folder_path = os.getcwd()
            destination_path = folder_path + "/" + str(batch_id) + "/" + filename
            move(filename, destination_path)
            count += 1

    data = {}
    nb_sequence = count - 30
    data['nb'] = nb_sequence
    with open(folder_path + "/" + str(batch_id) + "/" + "nb_sequence.txt", 'w') as outfile:  
        json.dump(data, outfile)
    

def draw_lines(batch_id):
    folder_path = get_path_to_batch(batch_id)
    file_path = folder_path + "/" + str(batch_id) + ".csv"


    folder_path = get_path_to_batch(batch_id)
    row, sort, box_array = remove_dupes(file_path, folder_path)

    for row in sort:
        box = Box(row)
        box_array.append(box)
    draw(box_array, batch_id)

class Box(object):
    def __init__(self, row):
        self.name = row[0]
        self.id = row[2]
        self.left = row[3]
        self.top = row[4]
        self.width = row[5]
        self.height = row[6]

def draw(box_array, counter):
    folder_path = get_path_to_batch(counter)
    current_frame = None
    image_reference = None
    is_first = True
    ids = []

    for box in box_array:
        formatted_frame = get_formatted_name(box.name)
        if is_first:
            try:
                image_reference = Image.open(folder_path + "/" + formatted_frame)
            except FileNotFoundError:
                print("Not found: " + formatted_frame)
                continue
            if image_reference is not None:
                current_frame = formatted_frame
                draw_box(image_reference, box)
                is_first = False
        else:
            # Only draw once per frame in case of duplicate boxes
            if current_frame == formatted_frame and image_reference is not None:
                if box.id not in ids:
                    draw_box(image_reference, box)
                    ids.append(box.id)

            else:
                if image_reference is not None:
                    image_reference.save(folder_path + "/" + "9" +current_frame)
                    image_reference = None
                    ids = []
                current_frame = formatted_frame
                try:
                    image_reference = Image.open(folder_path + "/" + formatted_frame)

                except FileNotFoundError:
                    print("Not found: " + formatted_frame)
                    continue

                if image_reference is not None:
                    draw_box(image_reference, box)
    if image_reference is not None:
        image_reference.save(folder_path + "/" + "9" +current_frame)

def draw_box(image, box):
    if image is not None:
        drawing = ImageDraw.Draw(image)
        top_left = (float(box.left), float(box.top))
        top_right = (float(box.left)+float(box.width), float(box.top))
        bottom_right = (float(box.left)+float(box.width), float(box.top)+float(box.height))
        bottom_left = (float(box.left), float(box.top)+float(box.height))
        line_color = (0, 255, 0)
        drawing.line([top_left, bottom_left, bottom_right, top_right, top_left], fill=line_color, width=2)

def remove_dupes(name, folder_path):
    rows = []

    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        ids = []
        cur_frame = 0
        for row in csv_reader:
            if row[0] == cur_frame:
                if row[2] in ids:
                    time.sleep(0)
                else:
                    ids.append(row[2])
                    rows.append(row)
            else: 
                ids = []
                cur_frame = row[0]
                ids.append(row[2])
                rows.append(row)
        csv_file.close()

    new_path = folder_path + "/corrected.csv"
    with open(new_path, 'w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(rows)
        csv_file.close()

    with open(new_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        box_array = []
        ids = []
        cur_frame = 0
        sort = sorted(csv_reader, key=operator.itemgetter(0))
        csv_file.close()

    with open(new_path, 'w') as myfile:
        writer = csv.writer(myfile)
        writer.writerows(sort)
        csv_file.close()    
    
    return row, sort, box_array

def main():

    missing_frames_array, first_frame, total = find_missing_frames()
    long_sequences = find_largest_batch(missing_frames_array, first_frame)
    counter = 0
    for sequence_id in long_sequences:
        copy_to_folder(int(sequence_id), int(long_sequences[sequence_id]), counter, total)
        #draw_lines(counter)
        counter += 1

if __name__ == '__main__':
    main()
