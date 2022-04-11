from dataclasses import dataclass


@dataclass
class RobotData:
    linear: float
    angular: float
    fl_height: float
    fr_height: float
    bl_height: float
    br_height: float

    def assignAllHeights(self, height: float) -> None:
        self.fl_height = height
        self.fr_height = height
        self.bl_height = height
        self.br_height = height

    def incrementAllHeights(self, delta: float) -> None:
        self.fl_height += delta
        self.fr_height += delta
        self.bl_height += delta
        self.br_height += delta

    def roundData(self) -> None:
        self.linear = round(self.linear, 1)
        self.angular = round(self.angular, 1)
        self.fl_height = round(self.fl_height, 1)
        self.fr_height = round(self.fr_height, 1)
        self.bl_height = round(self.bl_height, 1)
        self.br_height = round(self.br_height, 1)
