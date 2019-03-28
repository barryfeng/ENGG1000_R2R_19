class MazeAlgorithm:
    def case_straight(self):
        print("this is a straight")

    def case_corner(self):
        print("this is a corner")

    def case_tee(self):
        print("this is a tee")

    def case_four_junction(self):
        print("welp.")

    def determine_case(self, dist):
        if 200 <= dist:
            self.case_straight()
        elif dist <= 200:
            print('stop')
            self.case_corner()
        else:
            print("wot")