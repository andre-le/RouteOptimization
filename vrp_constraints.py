from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

#for http requests
import urllib.request
import json

import sys
from geopy.distance import vincenty

#######################
# Problem Constraints #
#######################
def vincenty_distance(pos_1, pos_2):
    pos_1 = (pos_1[0], pos_1[1])
    pos_2 = (pos_2[0], pos_2[1])
    return vincenty(pos_1, pos_2).meters

class CreateDistanceEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to return distance between points."""
    def __init__(self, data):
        """Initializes the distance matrix."""
        self._distances = {}
        self._violated_points = []

        # complete distance matrix
        # precompute distance between location to have distance callback in O(1)
        if data.distance_calculation == "OSRM":

            #filter out violated points
            url = "https://bi.ahamove.com/osrm/table/v1/driving/"
            for loc in data.locations:
                url += str(loc[1]) + "," + str(loc[0]) + ";"
            url = url[:-1] + "?annotations=distance"
            response = urllib.request.urlopen(url).read().decode('UTF-8')
            contents = json.loads(response)["distances"]

            if data.maximum_distance != 0:
                remove = []
                for index in xrange(data.num_locations):
                    min_distance = contents[index][0]
                    if min_distance > data.maximum_distance:
                        self._violated_points.append(data.locations[index])
                        remove.append(index)
                for index in sorted(remove, reverse=True):
                    data.remove_location(index)

            # only continue when there are more than 1 points in the dataset
            if len(data.locations) > 1:

                url = "https://bi.ahamove.com/osrm/table/v1/driving/"
                for loc in data.locations:
                    url += str(loc[1]) + "," + str(loc[0]) + ";"
                url = url[:-1] + "?annotations=distance"
                response = urllib.request.urlopen(url).read().decode('UTF-8')
                contents = json.loads(response)["distances"]
                
                if data.transport_mode == "N1":
                    for index in xrange(data.num_locations):
                        contents[0][index] = 0

                if data.transport_mode == "1N":
                    for index in xrange(data.num_locations):
                        contents[index][0] = 0

                self._distances = contents
        else:
            if data.maximum_distance != 0:
                remove = []
                for index in xrange(data.num_locations):
                    min_distance = (vincenty_distance(data.locations[0], data.locations[index]))
                    if min_distance > data.maximum_distance:
                        self._violated_points.append(data.locations[index])
                        remove.append(index)
                for index in sorted(remove, reverse=True):
                    data.remove_location(index)

            # only continue when there are more than 1 points in the dataset
            if len(data.locations) > 1:
                for from_node in xrange(data.num_locations):
                    self._distances[from_node] = {}
                    for to_node in xrange(data.num_locations):
                        # ignore distance from depot to others
                        # (we assign to driver that near the first point in the route)
                        if (from_node == to_node) or (data.transport_mode == "1N" and to_node == 0) or (data.transport_mode == "N1" and from_node == 0):
                            self._distances[from_node][to_node] = 0
                        else:
                            distance = (vincenty_distance(
                                    data.locations[from_node],
                                    data.locations[to_node]))
                            self._distances[from_node][to_node] = distance

    @property
    def get_violated_points(self):
        return self._violated_points

    def get_distance_matrix(self):
        return self._distances

    def distance_evaluator(self, from_node, to_node):
        return self._distances[from_node][to_node]

    # add cost if in cluster mode
    def cluster_distance_evaluator(self, from_node, to_node):
        # if distance more than 500m -> potential for crossing the river -> add more cost
        if self._distances[from_node][to_node] > 500:
            return self._distances[from_node][to_node] + 10000
        else:
            return self._distances[from_node][to_node]

def add_distance_dimension(routing, data, distance_evaluator):
    """Add Global Span constraint"""
    distance = "Distance"
    routing.AddDimension(
        distance_evaluator,
        0, # null slack
        data.maximum_distance, # maximum distance per vehicle
        True, # start cumul to zero
        distance)

def add_distance_soft(routing, data, distance_evaluator):
    distance = "Distance"
    routing.AddDimension(
        distance_evaluator,
        0, # null slack
        sys.maxsize, # maximum distance per vehicle
        True, # start cumul to zero
        distance)

    distance_dimension = routing.GetDimensionOrDie(distance)
    for vehicle_id in xrange(data.num_vehicles):
        distance_dimension.SetEndCumulVarSoftUpperBound(vehicle_id, data.maximum_distance, 10000)
    #distance_dimension.SetGlobalSpanCostCoefficient(data.num_locations*100)

class CreateParcelsEvaluator(object):
    """Creates callback to get parcels at each location."""
    def __init__(self, data):
        """Initializes the parcels array."""
        self._parcels = data.parcels

    def parcels_evaluator(self, from_node, to_node):
        del to_node
        return self._parcels[from_node]

def add_parcels_constraints(routing, data, parcels_evaluator):
    parcels = "Parcels"
    routing.AddDimension(
        parcels_evaluator,
        0, # null capacity slack
        data.max_parcels, # vehicle maximum capacity
        True, # start cumul to zero
        parcels)

    parcels_dimension = routing.GetDimensionOrDie(parcels)
    # cost = 0 if cumul > min_parcels
    if data.min_parcels != 0:
        for vehicle_id in xrange(data.num_vehicles):
            if data.maximum_distance == 0:
                parcels_dimension.SetEndCumulVarSoftLowerBound(vehicle_id, data.min_parcels, 10000000)
            else:
                parcels_dimension.SetEndCumulVarSoftLowerBound(vehicle_id, data.min_parcels, 10000) 