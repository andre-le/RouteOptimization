"""Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

#time for tester
import time

import csv
import random
import json

import pdvrp_cod_constraints as constraints
import pdvrp_cod_data_problem as data_problem
import pdvrp_cod_printer as printer

def return_lambda_gateway_response(code, body):
    return {"statusCode": code, "body": json.dumps(body)}

def get_routing_assignment(data, routing, assignment, distance_matrix, result_mode):
    cluster = []
    violated_cluster = []
    for vehicle_id in xrange(data.num_vehicles):
        if routing.IsVehicleUsed(assignment, vehicle_id):
            index = routing.Start(vehicle_id)
            index = assignment.Value(routing.NextVar(index))
            route_dist = 0
            route = []
            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                next_node_index = routing.IndexToNode(
                        assignment.Value(routing.NextVar(index)))
                route_dist += distance_matrix[node_index][next_node_index]
                if result_mode == "COORDINATES":         
                    route.append([data.locations[node_index][0], data.locations[node_index][1]])
                else:
                    route.append(data.orders_index[node_index])
                index = assignment.Value(routing.NextVar(index))

            node_index = routing.IndexToNode(index)
            if data.maximum_distance != 0 and route_dist > data.maximum_distance:
                violated_cluster.append(route)
            else:
                cluster.append(route)
    return {
        "cluster": cluster,
        "violated_cluster": violated_cluster 
    }

def handle(event, context):

    start_time = time.time()

    try:
        body = event.get('body')
        event = json.loads(body)

        depot = event["depot"]
        num_vehicles = event["vehicle_num"]
        orders = event["orders"]
        maximum_distance = event.get("max_distance", 0)
        maximum_parcels = event.get("vehicle_capacity", 20)
        distance_calculation = event.get("distance_calculation", "VINCENTY")
        result_mode = event.get("result_mode", "ORDERS")
        max_cod = event["max_cod"]

    except KeyError as e:
        print("Missing required input: " + str(e))
        cluster = {"title": "Missing required input: " + str(e)}
        return return_lambda_gateway_response(400, cluster)

    if maximum_distance < 0 or num_vehicles <= 0 or maximum_parcels <= 0 or max_cod <= 0:
        cluster = {"title": "Numerical input must be positive"}
        return return_lambda_gateway_response(400, cluster)

    if distance_calculation != "VINCENTY" and distance_calculation != "OSRM":
        cluster = {"title": "Invalid distance_calculation"}
        return return_lambda_gateway_response(400, cluster)

    if result_mode != "ORDERS" and result_mode != "COORDINATES":
        cluster = {"title": "Invalid result_mode"}
        return return_lambda_gateway_response(400, cluster)

    # Instantiate the data problem.
    data = data_problem.DataProblem(num_vehicles, depot, orders, maximum_distance, maximum_parcels, distance_calculation, max_cod)

    if distance_calculation == "OSRM" and data.num_locations > 100:
        cluster = {"title": "Error: OSRM reach the limit number of points. Please switch to Vincenty"}
        return return_lambda_gateway_response(400, cluster)

    # Define weight of each edge
    distance = constraints.CreateDistanceEvaluator(data)
    distance_matrix = distance.get_distance_matrix()
    distance_evaluator = distance.distance_evaluator
    parcels_evaluator = distance.parcels_evaluator
    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    
    if data.num_locations > 100:
        routing.SetArcCostEvaluatorOfAllVehicles(distance.cluster_distance_evaluator)
    else:
        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    constraints.add_pickup_delivery(routing, data)
    constraints.add_parcels_dimension(routing, data, parcels_evaluator)
    cod_evaluator = constraints.CreateCODEvaluator(data).cod_evaluator
    constraints.add_cod_constraints(routing, data, cod_evaluator)
    if maximum_distance != 0:
        constraints.add_distance_soft(routing, data, distance_evaluator)
    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.time_limit_ms = 10000
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment is None:
        print("No solution found")
        result = "No solution found"
    else:
        p = printer.ConsolePrinter(data, routing, assignment, distance_matrix)
        p.print()
        print("Cost: " + str(assignment.ObjectiveValue()))
        result = get_routing_assignment(data, routing, assignment, distance_matrix, result_mode)

    print("\nThe program took " + str(time.time() - start_time) + " seconds to run")

    return return_lambda_gateway_response(200, result)