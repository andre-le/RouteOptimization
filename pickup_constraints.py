from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

#for http requests
import urllib.request
import json

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
        if data.num_locations < 100:
            url = "https://bi.ahamove.com/osrm/table/v1/driving/"
            for loc in data.locations:
                url += str(loc[1]) + "," + str(loc[0]) + ";"
            url = url[:-1] + "?annotations=distance"
            response = urllib.request.urlopen(url).read().decode('UTF-8')
            contents = json.loads(response)
            self._distances = contents["distances"]
        else:
            for from_node in xrange(data.num_locations):
                self._distances[from_node] = {}
                for to_node in xrange(data.num_locations):
                    # ignore distance from other back to depot
                    if from_node == to_node or to_node == 0:
                        self._distances[from_node][to_node] = 0
                    else:
                        distance = (vincenty_distance(
                                data.locations[from_node],
                                data.locations[to_node]))
                        self._distances[from_node][to_node] = distance

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
            return self._distances[from_node][to_node] + 1000
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
        100000, # vehicle maximum capacity
        True, # start cumul to zero
        parcels)

    parcels_dimension = routing.GetDimensionOrDie(parcels)
    # cost = 0 if cumul > min_parcels
    for vehicle_id in xrange(data.num_vehicles):
        parcels_dimension.SetEndCumulVarSoftLowerBound(vehicle_id, data.min_parcels, 100000000) 