tags_dict = {
            0: 0,       # unlabeled --> noise
            1: 28,      # building --> static.manmade
            2: 28,      # fence --> static.manmade
            3: 29,      # other --> static.other
            4: 2,       # pedestrian --> human.pedestrian.adult
            5: 28,      # pole --> static.manmade
            6: 29,      # road line --> static.other
            7: 24,      # road --> flat.driveable_surface
            8: 26,      # sidewalk --> flat.sidewalk
            9: 30,      # vegetation --> static.vegetation
            10: 17,     # vehicles --> vehicle.car
            11: 28,     # wall --> static.manmade
            12: 28,     # traffic sign --> static.manmade
            13: 0,      # sky --> noise
            14: 25,     # ground --> flat.other
            15: 28,     # bridge --> static.manmade
            16: 28,     # rail track --> static.manmade
            17: 28,     # guard rail --> static.manmade
            18: 28,     # traffic light --> static.manmade
            19: 28,     # static --> static.manmade
            20: 11,     # dynamic --> movable_object.pushable_pullable
            21: 0,      # water --> noise
            22: 27,     # terrain --> flat.terrain
            23: 23,     # truck --> vehicle.truck
            24: 16,     # bus --> vehicle.bus.rigid
            25: 21,     # motorcycle --> vehicle.motorcycle
            26: 14,     # bicycle --> vehicle.bicycle
            27: 0,       # rider --> no label yet....
            28: 0
        }

attributes = {
    "vehicle.moving": "Vehicle is moving",
    "vehicle.parked": "Vehicle is parked",
    "pedestrian.moving": "The pedestrian is moving (assuming all are moving at all times)"
}

categories = [
    "noise",
    "animal",
    "human.pedestrian.adult",
    "human.pedestrian.child",
    "human.pedestrian.construction_worker",
    "human.pedestrian.personal_mobility",
    "human.pedestrian.police_officer",
    "human.pedestrian.stroller",
    "human.pedestrian.wheelchair",
    "movable_object.barrier",
    "movable_object.debris",
    "movable_object.pushable_pullable",
    "movable_object.trafficcone",
    "static_object.bicycle_rack",
    "vehicle.bicycle",
    "vehicle.bus.bendy",
    "vehicle.bus.rigid",
    "vehicle.car",
    "vehicle.construction",
    "vehicle.emergency.ambulance",
    "vehicle.emergency.police",
    "vehicle.motorcycle",
    "vehicle.trailer",
    "vehicle.truck",
    "flat.driveable_surface",
    "flat.other",
    "flat.sidewalk",
    "flat.terrain",
    "static.manmade",
    "static.other",
    "static.vegetation",
    "vehicle.ego"
]

visibilities = {
    "v0": "visibility of whole object is 0%",
    "v0-40": "visibility of whole object is between 0 and 40%",
    "v40-60": "visibility of whole object is between 40 and 60%",
    "v60-80": "visibility of whole object is between 60 and 80%",
    "v80-100": "visibility of whole object is between 80 and 100%",
}
