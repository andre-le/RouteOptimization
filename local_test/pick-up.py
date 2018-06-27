"""Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

#time for tester
import time

import csv
import random

import pickup_constraints

###########################
# Problem Data Definition #
###########################
def get_locations(num_of_locations):
	with open('location_sgn.csv', 'r', encoding="utf-8") as csvfile:
		mycsv = csv.reader(csvfile)
		rand_num = random.sample(range(0, 9500), num_of_locations)
		pick_rows=[row for idx, row in enumerate(mycsv) if idx in rand_num]
		for i, s in enumerate(pick_rows):
			pick_rows[i] = [float(s[1]), float(s[0]), s[2]]
		print("Data set: " + str(pick_rows))
		return pick_rows

class DataProblem():
    """Stores the data for the problem"""
    def __init__(self):
        """Initializes the data for the problem"""
        self._num_vehicles = 5
        self._maximum_distance = 45000
        self._min_parcels = 7
        self._num_locations = 300

        # self._locations = get_locations(self._num_locations)
        # self._parcels = [random.choice(range(1, 4)) for i in range(self._num_locations)]

        # Locations in block unit
        self._locations = \
                [
                 (10.7689049, 106.6637698), # depot, ahamove
                 (10.8083274, 106.7291283), # thao dien
                 (10.7748099, 106.700699), # dinh doc lap
                 (10.7634317, 106.6377053), # dam sen
                 (10.7719892, 106.7022025), # bitexco
                 (10.8043369, 106.708527), # dai hoc hutech
                 (10.8021978, 106.672429), # svd quan khu 7
                 (10.8308345, 106.6438679), # tan son nhat
                 (10.7831447, 106.702722), # dreamplex nguyen trung ngan
                 (10.7502831, 106.6483749), # ben xe cho lon
                 (10.8346252, 106.6743355), # dh tran dai nghia
                 (10.8368623, 106.7380292), # cau go dua
                 (10.8617355, 106.8001524), # suoi tien
                 (10.7674223, 106.6907763), # new world saigon
                 (10.7569914, 106.6788142), # dh saigon
                 (10.7472182, 106.6243638), # cong vien Phu Lam
                 (10.7772832, 106.6951255), # nha hat thanh pho
                 (10.7864613, 106.687713), # vien pasteur
                 (10.7783352, 106.6116854), # cho Binh Long
                 (10.8007547, 106.650517) # grand palace
		        ]
                 
        self._parcels = \
            [0, # depot, ahamove - 0
             2, # thao dien - 1
             5, # dinh doc lap - 2
             4, # dam sen - 3
             3, # bitexco - 4
             5, # dai hoc hutech - 5
             1, # svd quan khu 7 - 6
             1, # tan son nhat - 7
             2, # dreamplex nguyen trung ngan - 8
             3, # ben xe cho lon - 9
             3, # dh tran dai nghia - 10
             2, # cau go dua - 11
             3, # suoi tien - 12
             4, # new world saigon - 13
             2, # dh saigon - 14
             2, # cong vien Phu Lam - 15
             1, # nha hat thanh pho - 16
             1, # vien pasteur - 17
             3, # cho Binh Long - 18
             2 # grand palace - 19
            ]

        self._depot = 0

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        """Gets locations"""
        return self._locations

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot

    @property
    def parcels(self):
        """Gets parcels of each location"""
        return self._parcels

    @property
    def min_parcels(self):
        return self._min_parcels

    @property
    def maximum_distance(self):
        return self._maximum_distance

    # remove a point from locations list
    def remove_location(self, index):
        del self._locations[index]

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
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                route_dist += self.distance_matrix[node_index][next_node_index]
                route_load += self.data.parcels[node_index]
                plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)

                index = self.assignment.Value(self.routing.NextVar(index))

            node_index = self.routing.IndexToNode(index)
            total_dist += route_dist
            plan_output += ' {node_index}\n'.format(
                node_index=node_index)
            plan_output += 'Distance of the route {0}: {dist}'.format(
                vehicle_id,
                dist=route_dist)
            if (route_dist > self.data.maximum_distance):
                plan_output += "\t\tNOT SATISFY MAX DISTANCE"
            plan_output += '\nLoad of the route: {0}'.format(route_load)
            if (route_load <  self.data.min_parcels):
            	plan_output += "\t\tNOT SATISFY MIN PARCELS\n"
            print(plan_output)
        print('Total Distance of all routes: {dist}'.format(dist=total_dist))

########
# Main #
########
def main():

    start_time = time.time()
    """Entry point of the program"""
    # Instantiate the data problem.
    data = DataProblem()

    # Define weight of each edge
    distance = pickup_constraints.CreateDistanceEvaluator(data)
    distance_matrix = distance.get_distance_matrix()
    distance_evaluator = distance.distance_evaluator
    print("Violated points: " + str(distance.get_violated_points))
    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    
    if data.num_locations > 100:
        routing.SetArcCostEvaluatorOfAllVehicles(distance.cluster_distance_evaluator)
    else:
        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    pickup_constraints.add_distance_dimension(routing, data, distance_evaluator)
    parcels_evaluator = pickup_constraints.CreateParcelsEvaluator(data).parcels_evaluator
    pickup_constraints.add_parcels_constraints(routing, data, parcels_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.time_limit_ms = 10000
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    
    if True:
        routing.SetFixedCostOfAllVehicles(1000000)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
        print("change to soft constraint")
        routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)

        if data.num_locations > 100:
            routing.SetArcCostEvaluatorOfAllVehicles(distance.cluster_distance_evaluator)
        else:
            routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
        pickup_constraints.add_distance_soft(routing, data, distance_evaluator)
        parcels_evaluator = pickup_constraints.CreateParcelsEvaluator(data).parcels_evaluator
        pickup_constraints.add_parcels_constraints(routing, data, parcels_evaluator)

        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        search_parameters.time_limit_ms = 10000
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
        if True:
            routing.SetFixedCostOfAllVehicles(1000000)
        assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
    	print("No solution found")
    	cluster = "No solution found"
    else:
    	printer = ConsolePrinter(data, routing, assignment, distance_matrix)
    	printer.print()

    print("\nThe program took " + str(time.time() - start_time) + " seconds to run")

    return 0

if __name__ == '__main__':
  main()