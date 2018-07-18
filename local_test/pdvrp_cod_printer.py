from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

###########
# Printer #
###########
class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment, distance_matrix):
        """Initializes the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment
        self._distance_matrix = distance_matrix

    @property
    def data(self):
        """Gets problem data"""
        return self._data

    @property
    def routing(self):
        """Gets routing model"""
        return self._routing

    @property
    def assignment(self):
        """Gets routing model"""
        return self._assignment

    @property
    def distance_matrix(self):
        """Gets routing model"""
        return self._distance_matrix

    def print(self):
        """Prints assignment on console"""
        # Inspect solution.
        total_dist = 0
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            index = self.assignment.Value(self.routing.NextVar(index))
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                route_dist += self.distance_matrix[node_index][next_node_index]
                if node_index != 0 and node_index % 2 == 0:
                    route_load -= self.data.locations[node_index][2]
                if node_index % 2 != 0:
                    route_load += self.data.locations[node_index][2]
                plan_output += ' {0} Load({1}) -> '.format(self.data.orders_index[node_index], route_load)
                index = self.assignment.Value(self.routing.NextVar(index))
            node_index = self.routing.IndexToNode(index)
            total_dist += route_dist
            plan_output += '\nDistance of the route {0}: {dist}'.format(
                vehicle_id,
                dist=route_dist)
            print(plan_output)
        print('Total Distance of all routes: {dist}'.format(dist=total_dist))