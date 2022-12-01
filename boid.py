import random
import time
import numpy as np
# import PySimpleGUIWeb as sg
import PySimpleGUI as sg
from sklearn.preprocessing import normalize


class Boid():

    def __init__(self, count=50, centering=0.0005, align=0.05, avoid=0.05, perception=100, turn=0.1, speed_max=4, speed_min=3, width=800, height=800):
        # graph attributes
        self.iterations = 0
        self.width = width
        self.height = height
        self.margin = 75
        self.topmargin = height - self.margin
        self.bottommargin = self.margin
        self.rightmargin = width - self.margin
        self.leftmargin = self.margin

        self.boid_count = count
        self.limits = np.array([2000, 2000])
        self.positions = self.new_flock(self.boid_count, np.array([0, 0]), np.array([self.width, self.height]))
        self.velocities = self.new_flock(self.boid_count, np.array([0, -20]), np.array([10, 20]))
        self.separations = None
        self.square_distances = None

        # limits
        self.max_speed = speed_max
        self.min_speed = speed_min
        self.turnfactor = turn

        # parameters
        self.perception = perception
        self.alert_distance = self.perception ** 2  # for collision avoidance
        self.group_distance = self.perception ** 2  # for center of mass
        self.formation_flying_distance = self.perception ** 2  # for velocity matching
        self.move_to_middle_strength = centering  # for towards center of mass
        self.formation_flying_strength = align  # for velocity matching
        self.avoid_strength = avoid

    def reset_param(self, count=50, centering=0.0005, align=0.05, avoid=0.05, perception=100, turn=0.1, speed_max=4, speed_min=3, width=400, height=400):
        # graph attributes
        self.iterations = 0
        self.width = width
        self.height = height
        self.margin = 75
        self.topmargin = height - self.margin
        self.bottommargin = self.margin
        self.rightmargin = width - self.margin
        self.leftmargin = self.margin

        self.boid_count = count
        self.limits = np.array([2000, 2000])
        self.positions = self.new_flock(self.boid_count, np.array([0, 0]), np.array([self.width, self.height]))
        self.velocities = self.new_flock(self.boid_count, np.array([0, -20]), np.array([10, 20]))
        self.separations = None
        self.square_distances = None

        # limits
        self.max_speed = speed_max
        self.min_speed = speed_min
        self.turnfactor = turn

        # parameters
        self.perception = perception
        self.alert_distance = self.perception ** 2  # for collision avoidance
        self.group_distance = self.perception ** 2  # for center of mass
        self.formation_flying_distance = self.perception ** 2  # for velocity matching
        self.move_to_middle_strength = centering  # for towards center of mass
        self.formation_flying_strength = align  # for velocity matching
        self.avoid_strength = avoid

    def run(self, steps):
        """
        Runs the model for the specified number of steps
        :param steps:
        :return: the positions and velocities at the end
        """

        avg_order = 0
        steps_avg = 50
        for i in range(steps):
            self.update()
            if i >= steps - steps_avg:
                avg_order += self.polarization()
        avg_order = avg_order / steps_avg
        return avg_order

    def polarization(self):
        """
        Caluclates the polarization of the flock, a measure of global ordering. It represents how aligned the velocities of the individual birds are. The higher the number, the more aligned the birds
        are. A nonzero value indicates that the center of mass of the flock is moving. Bigger is better/more accurate for flocks
        :param input_flock:
        :return:
        """
        normed_vel = normalize(self.velocities, axis=0)
        sum_vel = np.sum(normed_vel, axis=1)
        sum_vel = sum_vel / self.boid_count
        order = np.linalg.norm(sum_vel)
        return order

    def reset_flock(self):
        """
        Resets the positions and the velocities to random values
        """
        self.positions = self.new_flock(self.boid_count, np.array([0, 0]), np.array([self.width, self.height]))
        self.velocities = self.new_flock(self.boid_count, np.array([0, -20]), np.array([10, 20]))

    def new_flock(self, count, lower_limits, upper_limits):
        width = upper_limits - lower_limits
        return lower_limits[:, np.newaxis] + np.random.rand(2, count) * width[:, np.newaxis]

    def update(self):
        self.apply_behaviour()

        # limit the speed
        vecs, norms = normalize(self.velocities, axis=0, return_norm=True)
        self.velocities[0, :][norms > self.max_speed] = vecs[0, :][norms > self.max_speed] * self.max_speed
        self.velocities[1, :][norms > self.max_speed] = vecs[1, :][norms > self.max_speed] * self.max_speed
        self.velocities[0, :][norms < self.min_speed] = vecs[0, :][norms < self.min_speed] * self.min_speed
        self.velocities[1, :][norms < self.min_speed] = vecs[1, :][norms < self.min_speed] * self.min_speed

        self.positions += self.velocities
        self.iterations += 1

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

    def edges(self):
        """
        keeps the birds in the frame by making them roll-over to the other side.
        """
        self.positions[self.positions >= self.width] = 0.001
        self.positions[self.positions <= 0.001] = self.width - 0.001
        self.velocities[self.positions < self.leftmargin] = (self.velocities + self.turnfactor + 1 / (self.positions))[self.positions < self.leftmargin]
        self.velocities[self.positions > self.rightmargin] = (self.velocities - self.turnfactor - 1 / (self.width - self.positions))[self.positions > self.rightmargin]
        np.nan_to_num(self.velocities)


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
        mean_velocities_differences = np.true_divide(np.sum(velocity_differences_if_close, 2), (velocity_differences_if_close != 0).sum(2), out=velocities,
                                                     where=(velocity_differences_if_close != 0).sum(2) != 0)

        self.velocities += mean_velocities_differences * self.formation_flying_strength

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
        centers_of_mass = np.true_divide(np.sum(locations_if_close, 2), (locations_if_close != 0).sum(2), out=centers_of_mass, where=(locations_if_close != 0).sum(2) != 0)  # get the mean x and y
        # positions

        direction_to_middle = centers_of_mass - self.positions  # change in position needed to be at the middle
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
        far_away = self.square_distances > self.alert_distance
        separations_if_close = np.copy(self.separations)  # separations is the change in position needed for the boid (in the row) to match the position of the other boid (in the column)
        separations_if_close[0, :, :][far_away] = 0
        separations_if_close[1, :, :][far_away] = 0

        # scale the differences by the inverse distance (dividing by the square distance essentially norms it then scales by the inverse distance)
        scaled_separations = np.copy(separations_if_close)
        scaled_separations = np.true_divide(separations_if_close, self.square_distances, out=scaled_separations, where=separations_if_close != 0)

        change = np.sum(scaled_separations, 1)
        self.velocities += change * self.avoid_strength


class Boid_visual():

    def __init__(self, count, window, width, height):
        # graph attributes
        self.pred_avoid_strength = 1
        self.iterations = 0
        self.width = width
        self.height = height
        self.margin = 75
        self.topmargin = height - self.margin
        self.bottommargin = self.margin
        self.rightmargin = width - self.margin
        self.leftmargin = self.margin

        self.boid_count = count
        self.limits = np.array([2000, 2000])
        self.positions = self.new_flock(self.boid_count, np.array([0, 0]), np.array([self.width, self.height]))
        self.velocities = self.new_flock(self.boid_count, np.array([0, -20]), np.array([10, 20]))
        self.separations = None
        self.square_distances = None

        # limits
        self.max_speed = 4
        self.min_speed = 3
        self.turnfactor = 0.1

        # parameters
        self.perception = 100
        self.alert_distance = self.perception ** 2  # for collision avoidance
        self.group_distance = self.perception ** 2  # for center of mass
        self.formation_flying_distance = self.perception ** 2  # for velocity matching
        self.move_to_middle_strength = 0.0005  # for towards center of mass
        self.formation_flying_strength = 0.05  # for velocity matching
        self.avoid_strength = 0.05

        # initialize graph ids
        graph = window.Element('_GRAPH_')  # type: sg.Graph
        self.drawing_ids = np.empty(self.boid_count, dtype=int)
        self.edges()
        for i, (x, y) in enumerate(zip(self.positions[0, :], self.positions[1, :])):
            self.drawing_ids[i] = graph.DrawCircle((x, y), radius=3, fill_color='black')

    def polarization(self):
        """
        Caluclates the polarization of the flock, a measure of global ordering. It represents how aligned the velocities of the individual birds are. The higher the number, the more aligned the birds
        are. A nonzero value indicates that the center of mass of the flock is moving. Bigger is better/more accurate for flocks
        :param input_flock:
        :return:
        """
        normed_vel = normalize(self.velocities, axis=0)
        sum_vel = np.sum(normed_vel, axis=1)

        sum_vel = sum_vel / self.boid_count

        order = np.linalg.norm(sum_vel)

        return order

    def new_flock(self, count, lower_limits, upper_limits):
        width = upper_limits - lower_limits
        return lower_limits[:, np.newaxis] + np.random.rand(2, count) * width[:, np.newaxis]

    def update(self):
        self.apply_behaviour()
        # limit the speed
        vecs, norms = normalize(self.velocities, axis=0, return_norm=True)
        self.velocities[0, :][norms > self.max_speed] = vecs[0, :][norms > self.max_speed] * self.max_speed
        self.velocities[1, :][norms > self.max_speed] = vecs[1, :][norms > self.max_speed] * self.max_speed
        self.velocities[0, :][norms < self.min_speed] = vecs[0, :][norms < self.min_speed] * self.min_speed
        self.velocities[1, :][norms < self.min_speed] = vecs[1, :][norms < self.min_speed] * self.min_speed


        self.positions += self.velocities
        self.iterations += 1

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
        # if self.iterations > 1000: #and self.iterations < 1200:
        #     self.object_avoid()

        self.edges()  # change position if it is off-screen

    def edges(self):
        """
        keeps the birds in the frame by making them roll-over to the other side.
        """
        # self.positions[self.positions == 0] = -0.0001
        # self.positions[self.positions == self.width] = self.width + 0.0001
        self.positions[self.positions >= self.width] = 0.001
        self.positions[self.positions <= 0.001] = self.width - 0.001
        self.velocities[self.positions < self.leftmargin] = (self.velocities + self.turnfactor + 1 / (self.positions))[self.positions < self.leftmargin]
        self.velocities[self.positions > self.rightmargin] = (self.velocities - self.turnfactor - 1 / (self.width - self.positions))[self.positions > self.rightmargin]
        np.nan_to_num(self.velocities)

        # for i in range(self.boid_count):
        #
        #     # turn when approaching edges
        #     # if self.positions[0, i] < self.leftmargin:
        #     #     self.velocities[0, i] = self.velocities[0, i] + self.turnfactor
        #     # if self.positions[0, i] > self.rightmargin:
        #     #     self.velocities[0, i] = self.velocities[0, i] - self.turnfactor
        #     # if self.positions[1, i] < self.bottommargin:
        #     #     self.velocities[1, i] = self.velocities[1, i] + self.turnfactor
        #     # if self.positions[1, i] > self.topmargin:
        #     #     self.velocities[1, i] = self.velocities[1, i] - self.turnfactor
        #
            # # turn when approaching edges
            # if self.positions[0, i] < self.leftmargin:
            #     distance = self.positions[0, i]
            #     if distance != 0:
            #         self.velocities[0, i] = self.velocities[0, i] + self.turnfactor + 1 / (distance)
            # if self.positions[0, i] > self.rightmargin:
            #     distance = self.width - self.positions[0, i]
            #     if distance != 0:
            #         self.velocities[0, i] = self.velocities[0, i] - self.turnfactor - 1 / (distance)
            # if self.positions[1, i] < self.bottommargin:
            #     distance = self.positions[1, i]
            #     if distance != 0:
            #         self.velocities[1, i] = self.velocities[1, i] + self.turnfactor + 1 / (distance)
            # if self.positions[1, i] > self.topmargin:
            #     distance = self.height - self.positions[1, i]
            #     if distance != 0:
            #         self.velocities[1, i] = self.velocities[1, i] - self.turnfactor - 1 / (distance)

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
        mean_velocities_differences = np.true_divide(np.sum(velocity_differences_if_close, 2), (velocity_differences_if_close != 0).sum(2), out=velocities,
                                                     where=(velocity_differences_if_close != 0).sum(2) != 0)

        # get the normed vectors and then make the magnitude equal the value of the parameter
        # for i in range(self.boid_count):
        #     if mean_velocities_differences[0, i] != 0 and mean_velocities_differences[1, i] != 0:
        #         mean_velocities_differences[:, i] = np.divide(mean_velocities_differences[:, i], np.linalg.norm(mean_velocities_differences[:, i]))

        self.velocities += mean_velocities_differences * self.formation_flying_strength

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
        centers_of_mass = np.true_divide(np.sum(locations_if_close, 2), (locations_if_close != 0).sum(2), out=centers_of_mass, where=(locations_if_close != 0).sum(2) != 0)  # get the mean x and y
        # positions

        direction_to_middle = centers_of_mass - self.positions  # change in position needed to be at the middle

        # Normalize and make the magnitude equal to that of the parameter
        # for i in range(self.boid_count):
        #     if direction_to_middle[0, i] != 0 and direction_to_middle[1, i] != 0:  # ensure that a change is needed
        #         direction_to_middle[:, i] = np.divide(direction_to_middle[:, i], np.linalg.norm(direction_to_middle[:, i])) * self.move_to_middle_strength

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
        far_away = self.square_distances > self.alert_distance
        separations_if_close = np.copy(self.separations)  # separations is the change in position needed for the boid (in the row) to match the position of the other boid (in the column)
        separations_if_close[0, :, :][far_away] = 0
        separations_if_close[1, :, :][far_away] = 0

        # scale the differences by the inverse distance (dividing by the square distance essentially norms it then scales by the inverse distance)
        scaled_separations = np.copy(separations_if_close)
        scaled_separations = np.true_divide(separations_if_close, self.square_distances, out=scaled_separations,  where=separations_if_close != 0)

        change = np.sum(scaled_separations, 1)

        # Normalize and make the magnitude equal to that of the parameter
        # for i in range(self.boid_count):
        #     if change[0, i] != 0 and change[1, i] != 0:  # ensure that a change is needed
        #         change[:, i] = np.divide(change[:, i], np.linalg.norm(change[:, i])) * self.avoid_strength

        self.velocities += change * self.avoid_strength

    def predator_dist(self):
        # separations is the change in position needed for the boid (in the row) to match the position of the other boid (in the column)
        self.predators_loc = np.full((2,self.boid_count), self.width / 2)  # make a predator in the middle of the screen
        pred_dist = self.positions - self.predators_loc  # distance from/relative to the predator
        squared_displacements = pred_dist * pred_dist
        scaled_separations = np.copy(pred_dist)
        scaled_separations = np.true_divide(1, pred_dist, out=scaled_separations, where=pred_dist != 0)

        self.velocities += scaled_separations * self.pred_avoid_strength

    def update_predator(self):
        one = np.ones((2, self.boid_count))
        rand = np.random.rand(2)
        one = one[0] * np.random.random()
        one = one[1] * np.random.random()
        self.predators_loc += one

    def object_avoid(self):
        self.predator_dist()
        self.update_predator()