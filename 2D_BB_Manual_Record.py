#!/usr/bin/env python

# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# This script is used as a client side script for CARLA simulator
# After launching the simulator server, this script allows the user to move 
# as a car in the simulation, and record sequences of images and bounding box data
# in the _out/ directory




"""
An example of client-side bounding boxes with basic car controls.
Controls:
    W            : throttle
    S            : brake
    AD           : steer
    Space        : hand-brake
    ESC          : quit
    R            : record frames to png and bounding boxes to csv
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
import logging
import copy
import weakref
import random
import re
import argparse
import collections
import datetime
import colorsys
import math
import time
import csv

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_r
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# View window constants
VIEW_WIDTH = 352
VIEW_HEIGHT = 352
VIEW_FOV = 90

# Probability of a car spawning
CAR_TRESHOLD = 0.4

# Number of walkers in simulation
NB_WALKERS = 100



# Bounding boxes constants
BB_COLOR = (248, 64, 24)
BB_COLOR2 = (24, 64, 248)
BB_COLOR3 = (24, 248, 64)

FRAMES = 500 # Changes the number of frames recorded to disk
MAX_DISTANCE = 30 # Distance treshold at which actors stop having bounding boxes

# Classes for different actors
VEHICLE_MAP = {
    "vehicle.audi.a2"                   : 2,
    "vehicle.audi.tt"                   : 2,
    "vehicle.audi.etron"                : 2,
    "vehicle.bmw.grandtourer"           : 2,
    "vehicle.bmw.isetta"                : 2,
    "vehicle.carlamotors.carlacola"     : 5,
    "vehicle.chevrolet.impala"          : 2,
    "vehicle.citroen.c3"                : 2,
    "vehicle.dodge_charger.police"      : 2,
    "vehicle.jeep.wrangler_rubicon"     : 2,
    "vehicle.lincoln.mkz2017"           : 2,
    "vehicle.mercedes-benz.coupe"       : 2,
    "vehicle.mini.cooperst"             : 2,
    "vehicle.ford.mustang"              : 2,
    "vehicle.nissan.micra"              : 2,
    "vehicle.nissan.patrol"             : 2,
    "vehicle.seat.leon"                 : 2,
    "vehicle.tesla.model3"              : 2,
    "vehicle.toyota.prius"              : 2,
    "vehicle.volkswagen.t2"             : 3,
    "vehicle.bh.crossbike"              : 1,
    "vehicle.harley-davidson.low rider" : 1,
    "vehicle.kawasaki.ninja"            : 1,
    "vehicle.yamaha.yzf"                : 1,
    "vehicle.gazelle.omafiets"          : 10,
    "vehicle.diamondback.century"       : 10,
    "walker.pedestrian.0001"            : 9,
    "walker.pedestrian.0002"            : 9,
    "walker.pedestrian.0003"            : 9,
    "walker.pedestrian.0004"            : 9,
    "walker.pedestrian.0005"            : 9,
    "walker.pedestrian.0006"            : 9,
    "walker.pedestrian.0007"            : 9,
    "walker.pedestrian.0008"            : 9,
    "walker.pedestrian.009"             : 9,
    "walker.pedestrian.010"             : 9,
    "walker.pedestrian.011"             : 9,
    "walker.pedestrian.012"             : 9,
    "walker.pedestrian.013"             : 9,
    "walker.pedestrian.014"             : 9
    }

# ==============================================================================
# -- ClientSideBoundingBoxes ---------------------------------------------------
# ==============================================================================


class ClientSideBoundingBoxes(object):

# ===============================================================================
# -- This is a module responsible for creating 3D bounding boxes and drawing them
# -- client-side on pygame surface
# ===============================================================================

    @staticmethod
    def get_bounding_boxes(vehicles, camera):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """
        id_bounding_boxes = [ClientSideBoundingBoxes.get_bounding_box(vehicle, camera) for vehicle in vehicles]
       
        # filter objects behind camera
        id_bounding_boxes = [bb for bb in id_bounding_boxes if (all(bb.bb[:, 2] > 0) and (all(bb.bb[:, 2] < MAX_DISTANCE)))]
        return id_bounding_boxes

    @staticmethod
    def is_contained(point):
        return point[0] > 0 and point[0] < VIEW_WIDTH and point[1] > 0 and point[1] < VIEW_HEIGHT

    @staticmethod
    def draw_bounding_boxes(display, bounding_boxes, recording, recording_number, real_frame):

        # Duplicated code again
        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        rows = []
        box_color = BB_COLOR if recording else BB_COLOR2

        for bbox in bounding_boxes:
            points = [(int(bbox.bb[i, 0]), int(bbox.bb[i, 1])) for i in range(8)]
            min_x = VIEW_WIDTH
            min_y = VIEW_HEIGHT
            max_x = 0
            max_y = 0

            for point in points:
                max_x = VIEW_WIDTH  if point[0] > VIEW_WIDTH  else point[0] if point[0] > max_x else max_x
                max_y = VIEW_HEIGHT if point[1] > VIEW_HEIGHT else point[1] if point[1] > max_y else max_y
                min_x = 0 if point[0] < 0 else point[0] if point[0] < min_x else min_x
                min_y = 0 if point[1] < 0 else point[1] if point[1] < min_y else min_y
            
            point_array = ([min_x, min_y], [min_x, max_y],[max_x, max_y], [max_x, min_y])

            has_contained_point = False
            
            for point in point_array:
                has_contained_point = True if ClientSideBoundingBoxes.is_contained(point) else has_contained_point
                
            if has_contained_point:
                index1 = 0
                index2 = 1
                for point in point_array:
                    pygame.draw.line(bb_surface, box_color, point_array[index1%len(point_array)], point_array[index2%len(point_array)])
                    index1 += 1
                    index2 += 1
                if recording:
                    rows.append([str(real_frame), VEHICLE_MAP[bbox.type], bbox.id, str(min_x), str(min_y), str(max_x-min_x), str(max_y-min_y)])
            
        if recording:
            with open('./_out/recordingNo'+str(recording_number)+'.csv', 'a') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerows(rows)
                csv_file.close()
        display.blit(bb_surface, (0, 0))

    @staticmethod
    def get_bounding_box(vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)

        id_coords = IdCoords(vehicle, camera_bbox)

        return id_coords

    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix

# ==============================================================================
# -- IdCoords ---------------------------------------------------
# ==============================================================================

class IdCoords(object):
    def __init__(self, actor, bb):
        self.type = actor.type_id
        self.id = actor.id
        self.bb = bb

# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self):
        self.client = None
        self.world = None
        self.camera = None
        self.car = None
        self.real_frame = 0

        self.display = None
        self.image = None
        self.capture = True
        self.recording = False
        self.frame_number = 0
        self.begin_frame = 0
        self.recording_number = 0
        self.id_list = []

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controled.
        """
        car_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        camera_transform = carla.Transform(carla.Location(x=1.6, z=1.7), carla.Rotation(pitch=-15))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

    def toggle_recording(self):
        """
        Toggles the recording of the sensor to disk
        """
        self.recording = not self.recording
        print("Recording: " + str(self.recording))
        if self.recording is False:
                self.recording_number += 1

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """

        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        if keys[K_r]:
            self.toggle_recording()
            self.begin_frame = self.frame_number
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False
        if self.recording:
            self.real_frame = self.image.frame_number
            self.image.save_to_disk('_out/%08d' % self.real_frame)

    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """

        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

    def spawn_npc(self):

        vehicles = self.world.get_blueprint_library().filter('vehicle.*')

        spawn_points = self.world.get_map().get_spawn_points()

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        for n, transform in enumerate(spawn_points):
            treshold = random.random()
            if treshold < CAR_TRESHOLD:
                blueprint = random.choice(vehicles)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                blueprint.set_attribute('role_name', 'autopilot')
                batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
        return batch

    def spawn_walkers(self):

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # Spawn Walkers
        spawn_points = []
        for i in range(NB_WALKERS):
            spawn_point = carla.Transform()
            spawn_point.location = self.world.get_random_location_from_navigation()
            spawn_points.append(spawn_point)
        batch = []
        info = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(self.world.get_blueprint_library().filter('walker.pedestrian.*'))
            batch.append(SpawnActor(walker_bp, spawn_point))

        # apply
        results = self.client.apply_batch_sync(batch, True)
        for i, result in enumerate(results):
            if result.error:
                logging.error(results[i].error)
            else:
                info.append({"id":result.actor_id, "trans":spawn_points[i], "con":None})

        # Spawn walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(info)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), info[i]["id"]))
        # apply
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                info[i]["con"] = results[i].actor_id

        # get whole list of actors (child and parents in same list so world.get_actors() can find parents also)
        all_id = []
        for i in range(len(info)):
            all_id.append(info[i]["con"])
            all_id.append(info[i]["id"])
        all_actors = self.world.get_actors(all_id)

        # initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            # index in the info list
            index = int(i / 2)
            all_actors[i].start(info[index]["trans"].location)

            # walk to random point
            target = self.world.get_random_location_from_navigation()
            all_actors[i].go_to_location(target)
            all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2

        print('spawned %d walkers, press Ctrl+C to exit.' % len(info))
        return info


    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            self.client = carla.Client('127.0.0.1', 2000) # IP, port
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()
            self.setup_car()
            self.setup_camera()
           
            batch = self.spawn_npc()

            # Send the batch to spawn NPCs
            for response in self.client.apply_batch_sync(batch):
                if response.error:
                    logging.error(response.error)
                else:
                    self.id_list.append(response.actor_id)

            # Spawn walkers and their controllers
            walkerInfo = self.spawn_walkers()

            for i in range(len(walkerInfo)):
                self.id_list.append((walkerInfo[i]["id"]))

            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)

            actors = self.world.get_actors(self.id_list)

            while True:
                self.world.tick()

                self.frame_number += 1

                self.capture = True
                pygame_clock.tick_busy_loop(15)

                self.render(self.display)
                bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(actors, self.camera)
                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bounding_boxes, self.recording, self.recording_number, self.real_frame)
                
                if self.frame_number-self.begin_frame >= FRAMES and self.recording:
                    self.toggle_recording()
                pygame.display.flip()

                pygame.event.pump()
                if self.control(self.car):
                    return

        finally:
            self.set_synchronous_mode(False)
            self.camera.destroy()
            self.car.destroy()
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.id_list])
            print('\ndestroying %d actors' % len(self.id_list))
            pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        client = BasicSynchronousClient()
        client.game_loop()
    finally:
        print('EXIT')


if __name__ == '__main__':
    main()