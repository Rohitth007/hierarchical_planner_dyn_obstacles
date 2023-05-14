import cv2

from matplotlib import pyplot as plt


class Map:
    def __init__(self, pgm_file="ed_3rd_floor_plan.pgm"):
        self.pgm = pgm_file
        self.resolution = 0.05
        self.origin = (-1.75, -9.9)  # 35, 198
        self.read_yaml()
        self.grid = cv2.imread(self.pgm, cv2.IMREAD_GRAYSCALE)
        # print(self.grid.shape)
        self.og = self.grid
        # self.show()

    def get_value(self, i, j):
        return self.grid[i, j]

    def read_yaml(self):
        pass

    def downsample(self, factor=1 / 8, cropi=(990, 1750), cropj=(950, 1830)):
        self.resolution *= 1 / factor
        # self.grid = cv2.imread(self.pgm, cv2.IMREAD_GRAYSCALE)
        self.grid = self.grid[cropi[0] : cropi[1], cropj[0] : cropj[1]]
        # self.show()
        ret, self.grid = cv2.threshold(self.grid, 100, 255, cv2.THRESH_BINARY)
        self.cropped = self.grid
        # self.show()

        self.grid = cv2.resize(
            self.grid,
            (0, 0),
            fx=factor,
            fy=factor,
            interpolation=cv2.INTER_AREA,
        )
        ret, self.grid = cv2.threshold(self.grid, 254, 255, cv2.THRESH_BINARY)
        # self.show()

        gauss = cv2.GaussianBlur(self.grid, (7, 7), 1)

        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i, j]:
                    self.grid[i, j] = gauss[i, j]
        # self.show()

    def show(self):
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(self.grid, cmap="gray")
        plt.show()

    def get_pos_from_indices(self, i, j):
        x = j * self.resolution + self.origin[0]
        height = len(self.grid)
        y = (height - i - 1) * self.resolution + self.origin[1]
        return x, y

    def get_indices_from_pos(self, x, y):
        j = (x - self.origin[0]) / self.resolution
        height = len(self.grid)
        i = (self.origin[1] - y) / self.resolution + height - 1
        return round(i), round(j)


if __name__ == "__main__":
    map = Map()
    map.downsample()
    map.show()
    print("map[0,0]:", map.get_value(0, 0))
    print((len(map.grid) - 1) // 2)
    print((len(map.grid[0]) - 1) // 2)
    print(map.get_pos_from_indices(0, 0))
    print(map.get_indices_from_pos(0, 0))
    print(map.get_pos_from_indices(561, 35))
    print(map.get_pos_from_indices(759, 0))
    print(map.get_pos_from_indices(0, 879))
    print(map.get_indices_from_pos(-1.75, -9.9))
    print(map.get_indices_from_pos(1.192, 1.347))
    print(map.get_pos_from_indices(5, 58))
