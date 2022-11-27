from p5 import Vector
import numpy as np
# import PySimpleGUIWeb as sg
import PySimpleGUI as sg


class Boid():

    def __init__(self, count, window, width, height):
        # graph attributes
        self.width = width
        self.height = height
        self.topmargin = height - 25
        self.bottommargin = 25
        self.rightmargin = width - 25
        self.leftmargin = 25
        self.turnfactor = 0.001

        self.boid_count = count
        self.limits = np.array([2000, 2000])
        self.positions = self.new_flock(self.boid_count, np.array([0, 0]), np.array([self.width, self.height]))
        self.velocities = self.new_flock(self.boid_count, np.array([0, -20]), np.array([10, 20]))
        self.separations = None
        self.square_distances = None

        # limits
        self.max_speed = 3

        # parameters
        self.alert_distance = 10 ** 2  # for collision avoidance
        self.group_distance = 75 ** 2  # for center of mass
        self.formation_flying_distance = 75 ** 2  # for velocity matching
        self.move_to_middle_strength = 1  # for towards center of mass
        self.formation_flying_strength = 5  # for velocity matching

        # initialize graph ids
        graph = window.Element('_GRAPH_')  # type: sg.Graph
        self.drawing_ids = np.empty(self.boid_count, dtype=int)
        self.edges()
        for i, (x, y) in enumerate(zip(self.positions[0, :], self.positions[1, :])):
            self.drawing_ids[i] = graph.DrawCircle((x, y), radius=3, fill_color='black')

        # self.acceleration = Vector(*vec)
        # self.max_force = 0.3
        # self.perception = 100
        #
        # self.drawing_id = None

    def new_flock(self, count, lower_limits, upper_limits):
        width = upper_limits - lower_limits
        return lower_limits[:, np.newaxis] + np.random.rand(2, count) * width[:, np.newaxis]

    def update(self):

        # limit the speed
        # self.velocities[self.velocities > self.max_speed] = (np.divide(self.velocities[0, :], np.linalg.norm(self.velocities, axis=0)) * self.max_speed)[too_fast]
        too_fast = np.linalg.norm(self.velocities, axis=0) > self.max_speed

        for i in range(self.boid_count):
            if np.linalg.norm(self.velocities[:, i]) > self.max_speed:
                self.velocities[:, i] = np.divide(self.velocities[:, i], np.linalg.norm(self.velocities[:, i])) * self.max_speed
        # new_x_vel = (np.divide(self.velocities[0, :], np.linalg.norm(self.velocities, axis=0)) * self.max_speed)[too_fast]
        # self.velocities[0, :][too_fast] = (np.divide(self.velocities[0, :], np.linalg.norm(self.velocities, axis=0)) * self.max_speed)[too_fast]
        # self.velocities[1, :][too_fast] = (np.divide(self.velocities[1, :], np.linalg.norm(self.velocities, axis=0)) * self.max_speed)[too_fast]

        # self.velocity += self.acceleration
        # self.acceleration = Vector(*np.zeros(2))
        self.positions += self.velocities


    def show(self, window):
        graph = window.Element('_GRAPH_')  # type: sg.Graph

        for x, y, id in zip(self.positions[0, :], self.positions[1, :], self.drawing_ids):
            graph.RelocateFigure(id, x, y)

    def apply_behaviour(self):
        """
        Combine and arbitrate the acceleration vectors given by applying each rule. "The acceleration request is in terms
         of a 3D vector that, by system convention, is truncated to unit magnitude or less."
        :param boids:
        :return:
        """
        self.calculate_distances()
        self.align()
        self.cohesion()
        self.separation()
        self.edges()  # change position if it is off-screen
        # alignment = self.align()
        # cohesion = self.cohesion()
        # separation = self.separation()

        # self.acceleration += alignment
        # self.acceleration += cohesion
        # self.acceleration += separation

    def edges(self):
        """
        keeps the birds in the frame by making them roll-over to the other side.
        """
        for i in range(self.boid_count):
            if self.positions[0, i] > self.width:
                self.positions[0, i] = 0
            elif self.positions[0, i] < 0:
                self.positions[0, i] = self.width

            if self.positions[1, i] > self.height:
                self.positions[1, i] = 0
            elif self.positions[1, i] < 0:
                self.positions[1, i] = self.height

            # turn when approaching edges
            if self.positions[0, i] < self.leftmargin:
                self.velocities[0, i] = self.velocities[0, i] + self.turnfactor
            if self.positions[0, i] > self.rightmargin:
                self.velocities[0, i] = self.velocities[0, i] - self.turnfactor
            if self.positions[1, i] > self.bottommargin:
                self.velocities[1, i] = self.velocities[1, i] - self.turnfactor
            if self.positions[1, i] < self.topmargin:
                self.velocities[1, i] = self.velocities[1, i] + self.turnfactor

    def calculate_distances(self):
        # separations is the change in position needed for the boid (in the row) to match the position of the other boid (in the column)
        self.separations = self.positions[:, np.newaxis, :] - self.positions[:, :, np.newaxis]
        squared_displacements = self.separations * self.separations
        self.square_distances = np.sum(squared_displacements, 0)

    """
    IMPORTANT
    if differences is 2xNxN
    this sums across the rows
        np.sum(differences, 2)
        
    this sums across the columns
        np.sum(differences, 1)
    """

    def align(self):
        """
        Rule: Velocity matching.
        A boid attempts to match the velocity of the boids around it
        This function calculates the difference between the boid and all the "close" boids
        :param boids: the list of all the boids in the flock
        :return: the acceleration( change velocity vector) needed to go at the max speed in the direction of all the "close" boids
        """
        velocity_differences = self.velocities[:, np.newaxis, :] - self.velocities[:, :, np.newaxis]  # the change in velocity needed to match the velocity of the other boid
        very_far = self.square_distances > self.formation_flying_distance
        velocity_differences_if_close = np.copy(velocity_differences)
        velocity_differences_if_close[0, :, :][very_far] = 0
        velocity_differences_if_close[1, :, :][very_far] = 0

        # get the mean velocity difference
        velocities = np.copy(self.velocities)
        mean_velocities_differences = np.true_divide(np.sum(velocity_differences_if_close, 2), (velocity_differences_if_close != 0).sum(2), out=velocities, where=(velocity_differences_if_close !=                                                                                                                                          0).sum(2) != 0)

        # get the normed vectors and then make the magnitude equal the value of the parameter
        for i in range(self.boid_count):
            if mean_velocities_differences[0, i] != 0 and mean_velocities_differences[1, i] != 0:
                mean_velocities_differences[:, i] = np.divide(mean_velocities_differences[:, i], np.linalg.norm(mean_velocities_differences[:, i])) * self.formation_flying_strength

        self.velocities += mean_velocities_differences

    def cohesion(self):
        """
        Rule: Flock centering.
        Get closer together. A boid moves towards the center of mass of all the "close" boids around it.
        Excludes each birds own position in calculation it's group's center of mass
        :param boids:
        :return: the change velocity vector needed to go at the max speed in the direction of the center of mass of all
                the "close" boids
        """
        # filter out the birds that are outside the perception area
        too_far = self.square_distances > self.group_distance
        locations_if_close = np.tile(self.positions[:, np.newaxis, :], (self.boid_count, 1))
        np.fill_diagonal(locations_if_close[0, :, :], 0)
        np.fill_diagonal(locations_if_close[1, :, :], 0)
        locations_if_close[0, :, :][too_far] = 0
        locations_if_close[1, :, :][too_far] = 0

        # get the mean positions of the nearby birds
        centers_of_mass = np.copy(self.positions)
        centers_of_mass = np.true_divide(np.sum(locations_if_close, 2), (locations_if_close != 0).sum(2), out=centers_of_mass,  where=(locations_if_close != 0).sum(2) != 0)  # get the mean x and y
        # positions

        direction_to_middle = centers_of_mass - self.positions  # change in position needed to be at the middle

        # Normalize and make the magnitude equal to that of the parameter
        for i in range(self.boid_count):
            if direction_to_middle[0, i] != 0 and direction_to_middle[1, i] != 0:  # ensure that a change is needed
                direction_to_middle[:, i] = np.divide(direction_to_middle[:, i], np.linalg.norm(direction_to_middle[:, i])) * self.move_to_middle_strength

        self.velocities += direction_to_middle

    def separation(self):
        """
        Rule: Collision avoidance.
        Avoid collision with other boids.
        We want the boid to steer away more from closer boids, compared to distant ones within their view (the circle).
        More force is exerted by the closer ones. We use the law of inverse distance
        :param boids:
        :return:
        """
        far_away = self.square_distances > self.alert_distance
        separations_if_close = np.copy(self.separations)  # separations is the change in position needed for the boid (in the row) to match the position of the other boid (in the column)
        separations_if_close[0, :, :][far_away] = 0
        separations_if_close[1, :, :][far_away] = 0
        sep = np.sum(separations_if_close, 1)  # gets the change needed to go in the opposite direction of the other boids
        self.velocities += np.sum(separations_if_close, 1)