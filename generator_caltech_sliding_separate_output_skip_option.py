import numpy as np
import keras
import matplotlib.pyplot as plt
import cv2
import os
import re

class DataGenerator(keras.utils.Sequence):
    
    def __init__(self, dataset_path, set_numbers, skip_number=10, skip_frames=3, batch_size=8,
                 sequence_length=15, prediction_length=5, image_shape=(88,88,3), shuffle=True):
        # Initialization
        self.sample_groups = []
        self.dataset_path = dataset_path
        self.set_numbers = set_numbers # list
        self.image_names = [ f for f in os.listdir(dataset_path) ]
        self.skip_number = skip_number
        self.skip_frames = skip_frames
        self.batch_size = batch_size
        self.sequence_length = sequence_length
        self.prediction_length = prediction_length
        self.image_shape = image_shape
        self.shuffle = shuffle
        self.create_sample_groups()
        print(self.sample_groups)
        self.on_epoch_end()
        
    def create_sample_groups(self):
        def atoi(text):
            return int(text) if text.isdigit() else text

        def natural_keys(text):
            return [ atoi(c) for c in re.split('(\d+)', text) ]

        for set_number in self.set_numbers:
            video_number = 0
            while os.path.isfile(self.dataset_path + 'set' + str(set_number).zfill(2)
                                 + '_V' + str(video_number).zfill(3) + '_0.jpg'):
                # filter images for current video
                r = re.compile("set" + str(set_number).zfill(2) + "_V" + str(video_number).zfill(3) + ".")
                images_list = list(filter(r.match, self.image_names))
                images_list.sort(key=natural_keys)
                
                # reduce frame rate
                images_list = images_list[0::self.skip_frames]
                
                # increase sliding window
                images_list = np.array(images_list)
                for i in range(1+len(images_list)-self.sequence_length):
                    if i%self.skip_number == 0:
                        self.sample_groups.append(images_list[i:i+self.sequence_length])
                video_number += 1
        
    def __len__(self):
        return int(np.floor(len(self.sample_groups) / self.batch_size))
        
    def __getitem__(self, index):
        # generate indexes for current batch
        indexes = self.indexes[index*self.batch_size:(index+1)*self.batch_size]
        
        # finds data for current batch
        sample_groups_for_batch = [self.sample_groups[k] for k in indexes]
        
        # Generate data
        X, Y = self.__data_generation(sample_groups_for_batch)
        
        return [X,Y], [Y,Y]
        
    def on_epoch_end(self):
        # Updates indexes after each epoch
        self.indexes = np.arange(len(self.sample_groups))
        if self.shuffle == True:
            np.random.shuffle(self.indexes)
            
    def __data_generation(self, sample_groups_for_batch):
        # Initialization
        
        X = np.empty((self.batch_size, self.sequence_length, *self.image_shape))
        Y = np.empty((self.batch_size, self.sequence_length, *self.image_shape))
        
        # Generate data
        for i, sample in enumerate(sample_groups_for_batch):
            for j, image in enumerate(sample):
                frame = cv2.imread(os.path.join(self.dataset_path,image))
                frame_resized = cv2.resize(frame, self.image_shape[:2], interpolation=cv2.INTER_AREA)
                
                if j < self.sequence_length - self.prediction_length:
                    X[i,j,] = frame_resized
                    Y[i,j,] = frame_resized
                elif j >= self.sequence_length - self.prediction_length:
                    X[i,j,] = np.zeros(self.image_shape)
                    Y[i,j,] = frame_resized
        
        X_norm = X.astype(np.uint8) / 255.
        Y_norm = Y.astype(np.uint8) / 255.
            
        return X_norm, Y_norm