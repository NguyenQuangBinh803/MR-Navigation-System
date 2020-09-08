__author__ = 'Edward J. C. Ashenbert'
import math

class PVector:

    def __init__(self, *args, **kwargs):
        if len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        else:
            self.x = 0
            self.y = 0

        self.isVector = True

    def add(self, a):
        try:
            self.x = self.x + a.x
            self.y = self.y + a.y
        except:
            pass

    def sub(self, b):
        try:
            self.x = self.x - a.x
            self.y = self.y - a.y
        except:
            pass

    def mag(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def heading(self):
        return math.atan2(self.y, self.x)

    def rotate(self, dislocation_angle):
        current_angle = self.heading()
        current_angle += dislocation_angle
        temp_mag = self.mag()

        self.x = temp_mag * math.cos(current_angle)
        self.y = temp_mag * math.sin(current_angle)

    def setMag(self, x):

        ratio = x / self.mag()
        self.x = self.x * ratio
        self.y = self.y * ratio
        # self.x = math.cos(self.heading()) * x
        # self.y = math.sin(self.heading()) * x

    def mult(self, x):
        self.x = self.x * x
        self.y = self.y * x

    def dot(self, vector):
        return self.x * vector.x + self.y * vector.y

    def set(self, x, y):
        self.x = x
        self.y = y

    def copy(self):
        return PVector(self.x, self.y)

    @staticmethod
    def add2Vectors(vector_a, vector_b):
        temp = PVector(vector_a.x + vector_b.x, vector_a.y + vector_b.y)
        return temp

    @staticmethod
    def sub2Vectors(vector_a, vector_b):
        temp = PVector(vector_a.x - vector_b.x, vector_a.y - vector_b.y)
        return temp


if __name__ == '__main__':
    a = PVector(2, 1)
    b = PVector(4, 1)
    b.add(a)
    c = PVector.add2Vectors(a, b)

    print(c.y)
