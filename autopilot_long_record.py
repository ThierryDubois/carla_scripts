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
VIEW_WIDTH = 800
VIEW_HEIGHT = 600
VIEW_FOV = 90

# Probability of a car spawning
CAR_TRESHOLD = 0.8

# Number of walkers in simulation
NB_WALKERS = 100


FRAMES = 2000 # Changes the number of frames recorded to disk

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
        settings.fixed_delta_seconds= 0.05
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
            self.image.save_to_disk('_out_long/%08d' % self.real_frame)

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
        all_id = []
        for i in range(NB_WALKERS):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
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
                logging.error(result.error)
            else:
                info.append({"id":result.actor_id})

        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(info)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), info[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                info[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(info)):
            all_id.append(info[i]["con"])
            all_id.append(info[i]["id"])
        all_actors = self.world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        self.world.wait_for_tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # random max speed
            all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)

        print('spawned %d walkers, press Ctrl+C to exit.' % (len(info)))

        return info

    def find_weather_presets(self):
        rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
        name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
        presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
        return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

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
            self.car.set_autopilot(True)
            preset = random.choice(self.find_weather_presets())
            self.world.set_weather(preset[0])
           
            batch = self.spawn_npc()

            # Send the batch to spawn NPCs
            for response in self.client.apply_batch_sync(batch):
                if response.error:
                    logging.error(response.error)
                else:
                    self.id_list.append(response.actor_id)

            # # Spawn walkers and their controllers
            # walkerInfo = self.spawn_walkers()

            # for i in range(len(walkerInfo)):
            #     self.id_list.append((walkerInfo[i]["id"]))

            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)

            while True:
                self.world.tick()

                self.frame_number += 1

                self.capture = True
                pygame_clock.tick_busy_loop(15)

                self.render(self.display)
                
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