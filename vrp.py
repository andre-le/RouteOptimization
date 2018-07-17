"""Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


import math
import json

#time for tester
import time

import data_problem
import printer
import vrp_constraints

def return_lambda_gateway_response(code, body):
    """
    This function wraps around the endpoint responses in a uniform and Lambda-friendly way
    :param code: HTTP response code (200 for OK), must be an int
    :param body: the actual content of the response
    """
    return {"statusCode": code, "body": json.dumps(body)}

def get_routing_assignment(data, routing, assignment, distance_matrix, violated_points):
    cluster = []
    violated_cluster = []
    for vehicle_id in xrange(data.num_vehicles):
        if routing.IsVehicleUsed(assignment, vehicle_id):
            index = routing.Start(vehicle_id)
            if data.transport_mode == "N1":
                index = assignment.Value(routing.NextVar(index))
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

            if data.transport_mode != "1N1":
                node_index = routing.IndexToNode(index)
                route.append([data.locations[node_index][0], data.locations[node_index][1]])
            if (data.maximum_distance != 0 and route_dist > data.maximum_distance) or (route_load < data.min_parcels):
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
    try:
        body = event.get('body')
        event = json.loads(body)

        locations = event["points"]
        min_parcels = event.get("min_parcels", 0)
        maximum_distance = event.get("max_distance", 0)
        num_vehicles = event["vehicle_num"]
        min_vehicles = event.get("min_vehicles", False)
        max_parcels = event.get("max_parcels", 20)
        transport_mode = event["transport_mode"]
        distance_calculation = event.get("distance_calculation", "VINCENTY")
    except KeyError as e:
        print("Missing required input: " + str(e))
        cluster = {"title": "Missing required input: " + str(e)}
        return return_lambda_gateway_response(400, cluster)

    if min_parcels < 0 or maximum_distance < 0 or num_vehicles < 0 or max_parcels < 0:
        cluster = {"title": "Numerical input cannot be negative"}
        return return_lambda_gateway_response(400, cluster)

    if transport_mode != "1N" and transport_mode != "N1" and transport_mode != "1N1":
        cluster = {"title": "Invalid transport_mode"}
        return return_lambda_gateway_response(400, cluster)

    if distance_calculation != "VINCENTY" and distance_calculation != "OSRM":
        cluster = {"title": "Invalid distance_calculation"}
        return return_lambda_gateway_response(400, cluster)

    if distance_calculation == "OSRM" and len(locations) > 100:
        cluster = {"title": "Bad request: OSRM cannot be used with more than 100 points"}
        return return_lambda_gateway_response(400, cluster)

    data = data_problem.DataProblem(locations, num_vehicles, min_parcels, 
        max_parcels, maximum_distance, transport_mode, distance_calculation)
    
    # Define weight of each edge
    distance = vrp_constraints.CreateDistanceEvaluator(data)
    distance_matrix = distance.get_distance_matrix()
    distance_evaluator = distance.distance_evaluator
    print("Violated points: " + str(distance.get_violated_points))

    if len(data.locations) <= 1:
        cluster = {
            "cluster": [],
            "violated_points": distance.get_violated_points,
            "violated_cluster": []
        }
        return return_lambda_gateway_response(200, cluster)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    
    if data.num_locations > 100:
        routing.SetArcCostEvaluatorOfAllVehicles(distance.cluster_distance_evaluator)
    else:
        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)

    if maximum_distance != 0:
        vrp_constraints.add_distance_dimension(routing, data, distance_evaluator)
    # still need when min_parcels = 0 because we have max_parcels
    parcels_evaluator = vrp_constraints.CreateParcelsEvaluator(data).parcels_evaluator
    vrp_constraints.add_parcels_constraints(routing, data, parcels_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.time_limit_ms = 25000
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)

    #minimize the total number of vehicle
    if min_vehicles:
        if data.num_vehicles*data.min_parcels >= data.num_locations:
            routing.SetFixedCostOfAllVehicles(1000000)
        else:
            routing.SetFixedCostOfAllVehicles(10000)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
        print("change distance to soft constraint")
        print("\nThe program took " + str(time.time() - start_time) + " seconds to run")
        start_time = time.time()
        routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)

        if data.num_locations > 100:
            routing.SetArcCostEvaluatorOfAllVehicles(distance.cluster_distance_evaluator)
        else:
            routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
        if maximum_distance != 0:
            vrp_constraints.add_distance_soft(routing, data, distance_evaluator)
        parcels_evaluator = vrp_constraints.CreateParcelsEvaluator(data).parcels_evaluator
        vrp_constraints.add_parcels_constraints(routing, data, parcels_evaluator)

        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        search_parameters.time_limit_ms = 60000
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)
        if min_vehicles:
            if data.num_vehicles*data.min_parcels >= data.num_locations:
                routing.SetFixedCostOfAllVehicles(1000000)
            else:
                routing.SetFixedCostOfAllVehicles(100)
        assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
        print("No solution found")
        cluster = "No solution found"
    else:
        cluster = get_routing_assignment(data, routing, assignment, distance_matrix, distance.get_violated_points)
        p = printer.ConsolePrinter(data, routing, assignment, distance_matrix)
        p.print()

    print("\nThe program took " + str(time.time() - start_time) + " seconds to run")

    return return_lambda_gateway_response(200, cluster)