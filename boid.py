from p5 import Vector
import numpy as np
# import PySimpleGUIWeb as sg
import PySimpleGUI as sg

class Boid():

    def __init__(self, x, y, width, height):
        self.position = Vector(x, y)
        vec = (np.random.rand(2) - 0.5)*10  # creates a random 2d numpy array
        self.velocity = Vector(*vec)

        vec = (np.random.rand(2) - 0.5)/2  # creates a random 2d numpy array
        self.acceleration = Vector(*vec)
        self.max_force = 0.3
        self.max_speed = 5
        self.perception = 100

        self.width = width
        self.height = height
        self.drawing_id = None


    def update(self):
        self.position += self.velocity
        self.velocity += self.acceleration
        # limit the speed
        if np.linalg.norm(self.velocity) > self.max_speed:
            self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.max_speed

        self.acceleration = Vector(*np.zeros(2))

    def show(self, window):
        graph = window.Element('_GRAPH_')       # type: sg.Graph
        if self.drawing_id is None:
            self.drawing_id = graph.DrawCircle((self.position.x, self.position.y), radius=3, fill_color='black')
        else:
            graph.RelocateFigure(self.drawing_id, self.position.x, self.position.y)


    def apply_behaviour(self, boids):
        """
        Combine and arbitrate the acceleration vectors given by applying each rule. "The acceleration request is in terms
         of a 3D vector that, by system convention, is truncated to unit magnitude or less."
        :param boids:
        :return:
        """
        alignment = self.align(boids)
        cohesion = self.cohesion(boids)
        separation = self.separation(boids)

        self.acceleration += alignment
        self.acceleration += cohesion
        self.acceleration += separation

    def edges(self):
        """
        keeps the birds in the frame by making them roll-over to the other side.
        """
        if self.position.x > self.width:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = self.width

        if self.position.y > self.height:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = self.height

    def align(self, boids):
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

        steering = Vector(*np.zeros(2))  # initialize steering vector. This vector represents the acceleration needed to
        # conform to the rule
        total = 0
        avg_vector = Vector(*np.zeros(2))
        for boid in boids:
            # if the boid in the iteration is within the perception distance of self.boid,
            if np.linalg.norm(boid.position - self.position) < self.perception:
                # add the velocity vector of the boid to avg_vector to get the total of the velocities of all the
                # "close" boids
                avg_vector += boid.velocity
                total += 1
        if total > 0:
            avg_vector /= total  # divide avg_vector by the total number of "close" boids
            avg_vector = Vector(*avg_vector)
            # normalize the avg_vector (make it a unit/directional vector) then make it the size of max_speed
            avg_vector = (avg_vector / np.linalg.norm(avg_vector)) * self.max_speed
            # get the difference between the average velocity of all the "close" boids and the velocity of the boid
            # in order to get the change needed to go in the direction of all the "close" boids
            steering = avg_vector - self.velocity

        return steering

    def cohesion(self, boids):
        """
        Rule: Flock centering.
        Get closer together. A boid moves towards the center of mass of all the "close" boids around it.
        :param boids:
        :return: the change velocity vector needed to go at the max speed in the direction of the center of mass of all
                the "close" boids
        """
        steering = Vector(*np.zeros(2))
        total = 0
        center_of_mass = Vector(*np.zeros(2))
        for boid in boids:
            # if the boid in the iteration is within the perception distance of self.boid,
            #  add the position vector to the total position
            if np.linalg.norm(boid.position - self.position) < self.perception:
                center_of_mass += boid.position
                total += 1
        if total > 0:
            center_of_mass /= total  # get the average position vector of all the "close" boids
            center_of_mass = Vector(*center_of_mass)
            # calculate the direction of the center of mass relative to the boids position
            vec_to_com = center_of_mass - self.position
            if np.linalg.norm(vec_to_com) > 0:  # if the boid is not at the center of mass
                # get the velocity vector in the direction of the center of mass at max speed
                vec_to_com = (vec_to_com / np.linalg.norm(vec_to_com)) * self.max_speed
            steering = vec_to_com - self.velocity
            if np.linalg.norm(steering)> self.max_force:  # bound the steering vector
                steering = (steering /np.linalg.norm(steering)) * self.max_force

        return steering

    def separation(self, boids):
        """
        Rule: Collision avoidance.
        Avoid collision with other boids.
        :param boids:
        :return:
        """
        steering = Vector(*np.zeros(2))
        total = 0
        avg_vector = Vector(*np.zeros(2))
        for boid in boids:
            distance = np.linalg.norm(boid.position - self.position)  # get distance to boid in iteration
            # if the iteration boid is within the perception distance
            if self.position != boid.position and distance < self.perception:
                diff = self.position - boid.position
                diff /= distance
                avg_vector += diff
                total += 1
        if total > 0:
            avg_vector /= total
            avg_vector = Vector(*avg_vector)
            if np.linalg.norm(steering) > 0:
                avg_vector = (avg_vector / np.linalg.norm(steering)) * self.max_speed
            steering = avg_vector - self.velocity
            if np.linalg.norm(steering) > self.max_force:
                steering = (steering /np.linalg.norm(steering)) * self.max_force

        return steering