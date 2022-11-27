from p5 import Vector
import numpy as np
# import PySimpleGUIWeb as sg
import PySimpleGUI as sg


class Boid():

    def __init__(self, count, window, width, height):
        self.boid_count = count
        self.width = width
        self.height = height
        self.limits = np.array([2000, 2000])
        self.positions = self.new_flock(self.boid_count, np.array([0, 0]), np.array([self.width, self.height]))
        self.velocities = self.new_flock(self.boid_count, np.array([0, -20]), np.array([10, 20]))
        self.separations = None
        self.square_distances = None

        # limits
        self.max_speed = 5

        # parameters
        self.alert_distance = 100  # for collision avoidance
        self.group_distance = 10000  # for center of mass
        self.formation_flying_distance = 10000  # for velocity matching
        self.move_to_middle_strength = 0.01  # for towards center of mass
        self.formation_flying_strength = 0.125  # for velocity matching

        # initialize graph ids
        graph = window.Element('_GRAPH_')  # type: sg.Graph
        self.drawing_ids = np.empty(self.boid_count)
        self.edges()
        ID = graph.DrawCircle((100.5, 56.8), radius=3, fill_color='black')
        for i, (x, y) in enumerate(zip(self.positions[0, :], self.positions[1, :])):
            id = graph.DrawCircle((x, y), radius=3, fill_color='black')
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
        self.positions += self.velocities

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
        for x, y in zip(self.positions[0, :], self.positions[1, :]):
            if x > self.width:
                x = 0
            elif x < 0:
                x = self.width

            if y > self.height:
                y = 0
            elif y < 0:
                y = self.height

    def calculate_distances(self):
        self.separations = self.positions[:, np.newaxis, :] - self.positions[:, :, np.newaxis]
        squared_displacements = self.separations * self.separations
        self.square_distances = np.sum(squared_displacements, 0)

    def align(self):
        """
        Rule: Velocity matching.
        A boid attempts to match the velocity of the boids around it
        This function calculates the difference between the boid and all the "close" boids
        :param boids: the list of all the boids in the flock
        :return: the acceleration( change velocity vector) needed to go at the max speed in the direction of all the "close" boids
        """
        # TODO: all of this makes the boids want to go in the direction of "close" boids, but at the max speed,
        #  rather than attempting to match the speed of the "close" boids. Should this be different? It could also just
        #  preserve the magnitude of the velocity of the boid and just match the angle

        far_away = self.square_distances > self.alert_distance
        separations_if_close = np.copy(self.separations)
        separations_if_close[0, :, :][far_away] = 0
        separations_if_close[1, :, :][far_away] = 0
        self.velocities += np.sum(separations_if_close, 1)

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
        centers_of_mass = np.copy(self.positions)
        centers_of_mass = np.true_divide(np.sum(locations_if_close, 2), (locations_if_close != 0).sum(1),  where=(locations_if_close != 0).sum(1) != 0)  # get the mean x and y positions
        direction_to_middle = centers_of_mass - self.positions  # change in position needed to be at the middle
        # TODO: it would make sense to normalize this
        self.velocities += direction_to_middle * self.move_to_middle_strength

    def separation(self):
        """
        Rule: Collision avoidance.
        Avoid collision with other boids.
        We want the boid to steer away more from closer boids, compared to distant ones within their view (the circle).
        More force is exerted by the closer ones. We use the law of inverse distance
        :param boids:
        :return:
        """
        velocity_differences = self.velocities[:, np.newaxis, :] - self.velocities[:, :, np.newaxis]
        very_far = self.square_distances > self.formation_flying_distance
        velocity_differences_if_close = np.copy(velocity_differences)
        velocity_differences_if_close[0, :, :][very_far] = 0
        velocity_differences_if_close[1, :, :][very_far] = 0
        self.velocities -= np.mean(velocity_differences_if_close, 1) * self.formation_flying_strength
