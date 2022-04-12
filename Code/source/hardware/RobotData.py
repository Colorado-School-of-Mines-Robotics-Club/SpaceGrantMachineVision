from dataclasses import dataclass


@dataclass
class RobotData:
    linear: float
    angular: float
    fl_height: float
    fr_height: float
    bl_height: float
    br_height: float

    def incrementAllHeights(self, delta):
        self.fl_height += delta
        self.fr_height += delta
        self.bl_height += delta
        self.br_height += delta

    def roundData(self):
        self.linear = round(self.linear, 1)
        self.angular = round(self.angular, 1)
        self.fl_height = round(self.fl_height, 1)
        self.fr_height = round(self.fr_height, 1)
        self.bl_height = round(self.bl_height, 1)
        self.br_height = round(self.br_height, 1)

    def from_list(self, data):
        self.linear = data[0]
        self.angular = data[1]
        self.fl_height = data[2]
        self.fr_height = data[3]
        self.bl_height = data[4]
        self.br_height = data[5]

    def to_list(self):
        return [self.linear, self.angular, self.fl_height, self.fr_height, self.bl_height, self.br_height]
