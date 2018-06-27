"""Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


import math

#time for tester
import time

import data_problem
import printer
import pickup_constraints

def get_routing_assignment(data, routing, assignment, distance_matrix, violated_points):
    cluster = []
    violated_cluster = []
    for vehicle_id in xrange(data.num_vehicles):
        index = routing.Start(vehicle_id)
        route_dist = 0
        route_load = 0
        route = []
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(
                    assignment.Value(routing.NextVar(index)))
            route_dist += distance_matrix[node_index][next_node_index]
            route_load += data.parcels[node_index]            
            route.append([data.locations[node_index][0], data.locations[node_index][1]])
            index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        route.append([data.locations[node_index][0], data.locations[node_index][1]])
        if route_load < data.min_parcels or route_dist > data.maximum_distance:
            violated_cluster.append(route)
        else:
            cluster.append(route)
    return {
        "cluster": cluster,
        "violated_points": violated_points,
        "violated_cluster": violated_cluster 
    }

def handle(event, context):
    start_time = time.time()
    """Entry point of the program"""
    # Instantiate the data problem.
    locations = event["points"]
    min_parcels = event["min_parcels"]
    maximum_distance = event["max_distance"]
    num_vehicles = event.get("vehicle_nums", math.ceil(maximum_distance/min_parcels))
    min_vehicles = event.get("min_vehicles", False)
    data = data_problem.DataProblem(locations, num_vehicles, min_parcels, maximum_distance)
    
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
    search_parameters.time_limit_ms = 25000
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)

    # minimize the total number of vehicle
    if min_vehicles:
        routing.SetFixedCostOfAllVehicles(1000000)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
        print("change distance to soft constraint")
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
        if min_vehicles:
            routing.SetFixedCostOfAllVehicles(1000000)
        assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
        print("No solution found")
        cluster = "No solution found"
    else:
        cluster = get_routing_assignment(data, routing, assignment, distance_matrix, distance.get_violated_points)
        p = printer.ConsolePrinter(data, routing, assignment, distance_matrix)
        p.print()

    print("\nThe program took " + str(time.time() - start_time) + " seconds to run")

    return cluster