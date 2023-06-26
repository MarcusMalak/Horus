
class SensorSpawn:
    def __init__(self, carla, world, world_bp, vehicle):
        self.carla = carla
        self.world = world
        self.world_bp = world_bp
        self.vehicle = vehicle

    def spawn_vehicle_cam(self):
        cam_bp = self.world_bp.find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(1920))
        cam_bp.set_attribute("image_size_y", str(1080))
        cam_bp.set_attribute("fov", str(105))
        # cam_bp.set_attribute('sensor_tick', 1.0)
        cam_location = self.carla.Location(2, 0, 1)
        cam_rotation = self.carla.Rotation(0, 0, 0)
        cam_transform = self.carla.Transform(cam_location, cam_rotation)
        vehicle_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.vehicle,attachment_type=self.carla.AttachmentType.Rigid)

        return vehicle_cam, cam_bp

    def spawn_vehicle_lidar(self):
        lidar_bp = self.world_bp.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(64))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(50))
        lidar_location = self.carla.Location(0, 0, 2)
        lidar_rotation = self.carla.Rotation(0, 0, 0)
        lidar_transform = self.carla.Transform(lidar_location, lidar_rotation)
        vehicle_lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)

        return vehicle_lidar

    def spawn_drone_cam(self, drone_height):
        drone_cam_bp = self.world_bp.find('sensor.camera.rgb')
        drone_cam_bp.set_attribute("image_size_x", str(1920))
        drone_cam_bp.set_attribute("image_size_y", str(1080))
        drone_cam_bp.set_attribute("fov", str(105))
        cam_top_location = self.carla.Location(2, 0, drone_height)
        cam_top_rotation = self.carla.Rotation(-90, 0, 0)
        cam_top_transform = self.carla.Transform(cam_top_location, cam_top_rotation)
        drone_cam = self.world.spawn_actor(drone_cam_bp, cam_top_transform, attach_to=self.vehicle, attachment_type=self.carla.AttachmentType.Rigid)

        return drone_cam, drone_cam_bp



