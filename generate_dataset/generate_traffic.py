import random
import carla
import time


class Traffic:
    def __init__(self, world, world_bp, client, map, num_vehicles, num_pedestrians, seed):
        self.world = world
        self.world_bp = world_bp
        self.client = client
        self.map = map
        self.num_vehicles = num_vehicles
        self.num_pedestrians = num_pedestrians

        # Set up the TM in synchronous mode
        self.traffic_manager = client.get_trafficmanager()

        self.traffic_manager.set_synchronous_mode(True)

        # Set a seed so behaviour can be repeated if necessary
        self.traffic_manager.set_random_device_seed(seed)

        self.spawn_points = map.get_spawn_points()
        random.seed(seed)

    def spawn_vehicles(self):
        # Select some models from the blueprint library
        blueprints = []

        # for vehicle in self.world_bp.filter('*vehicle*'):
        #     if any(model in vehicle.id for model in models):
        #         blueprints.append(vehicle)

        for vehicle in self.world_bp.filter('*vehicle*'):
            blueprints.append(vehicle)

        for vehicle_bp in blueprints:
            if vehicle_bp.has_attribute('color'):
                color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
                vehicle_bp.set_attribute('color', color)

        # for vehicle_bp in blueprints:
        #     print(vehicle_bp.id)
        #     if vehicle_bp.has_attribute('color'):
        #         print(vehicle_bp.get_attribute('color'))

        vehicles = []

        # while len(vehicles) < self.num_vehicles:
        # Set a max number of vehicles and prepare a list for those we spawn
        max_vehicles = self.num_vehicles - len(vehicles)
        vehicles_to_spawn = min([max_vehicles, len(self.spawn_points)])

        # Take a random sample of the spawn points and spawn some vehicles
        for i, spawn_point in enumerate(random.sample(self.spawn_points, vehicles_to_spawn)):
            temp = self.world.try_spawn_actor(random.choice(blueprints), spawn_point)
            if temp is not None:
                vehicles.append(temp)

        # Parse the list of spawned vehicles and give control to the TM through set_autopilot()
        for vehicle in vehicles:
            vehicle.set_autopilot(True)
            # Randomly set the probability that a vehicle will ignore traffic lights
            # self.traffic_manager.ignore_lights_percentage(vehicle, random.randint(0, 50))

            # self.spawn_points = self.map.get_spawn_points()

        # print('\nTraffic spawned')
        print(f'SPAWNED VEHICLES= {len(vehicles)}')

        return vehicles

    def spawn_pedestrians(self):
        bp_pedestrians = []
        max_pedestrians = min([self.num_pedestrians, len(self.spawn_points)])

        for pedestrian in self.world_bp.filter('*walker*'):
            bp_pedestrians.append(pedestrian)

        spawn_points = []

        for i in range(self.num_pedestrians):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        print(f'SPAWNED POINTS: {len(spawn_points)}')

        actors = []
        spawn_test = []
        # Spawn walker actors
        for i, spawn_point in enumerate(random.sample(self.spawn_points, max_pedestrians)):
            walker_bp = random.choice(bp_pedestrians)
            temp = self.world.try_spawn_actor(walker_bp, spawn_point)
            if temp is not None:
                actors.append(temp)
                spawn_test.append(spawn_point)
        print(f'SPAWNED PEDESTRIANS: {len(actors)}')

        controllers = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(actors)):
            temp = self.world.try_spawn_actor(walker_controller_bp, carla.Transform(), actors[i])
            controllers.append(temp)

        print(f'SPAWNED CONTROLLERS: {len(controllers)}')

        # print(f'num_walkers = ', len(actors))

        for i in range(len(actors)):
            controllers[i].start()

            loc = self.world.get_random_location_from_navigation()
            loc.x = spawn_test[i].location.x
            loc.y = spawn_test[i].location.y
            loc.z = spawn_test[i].location.z

            print(f'location: {loc}')
            controllers[i].go_to_location(loc)
            # loc = self.world.get_random_location_from_navigation()
            # print(f'control loc: {loc}')
            # print(f'controller: {controllers[i]}')
            # controllers[i].go_to_location(loc)
            print(f'started controller: {i}')

        print('\nWalkers spawned')

        return actors, controllers




