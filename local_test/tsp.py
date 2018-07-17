from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

import urllib.request
import json
from geopy.distance import vincenty

test_20 = {
    "points": [[10.773687, 106.703263], [10.731158, 106.716759], [10.729461, 106.714041], [10.768337, 106.700743], [10.827278, 106.678072], [10.772264, 106.681347], [10.786769, 106.640134], [10.875387, 106.755127], [10.808667, 106.711705], [10.774575, 106.705748], [10.827971, 106.727006], [10.770907, 106.6681], [10.769285, 106.674728], [10.737721, 106.675189], [10.786519, 106.693997], [10.798453, 106.667866], [10.772691, 106.693676], [10.783066, 106.695901], [10.754833, 106.66052], [10.770541, 106.703162]],
  "transport_mode": "1N1",
  "distance_calculation": "VINCENTY"
}


def return_lambda_gateway_response(code, body):
    return {"statusCode": code, "body": json.dumps(body)}

def vincenty_distance(pos_1, pos_2):
    pos_1 = (pos_1[0], pos_1[1])
    pos_2 = (pos_2[0], pos_2[1])
    return vincenty(pos_1, pos_2).meters

def create_distance_matrix(locations, transport_mode, distance_calculation):
# Create the distance matrix.
  dist_matrix = {}

  # complete distance matrix
  # precompute distance between location to have distance callback in O(1)
  if distance_calculation == "OSRM":
    url = "https://bi.ahamove.com/osrm/table/v1/driving/"
    for loc in locations:
      url += str(loc[1]) + "," + str(loc[0]) + ";"
    url = url[:-1] + "?annotations=distance"
    response = urllib.request.urlopen(url).read().decode('UTF-8')
    contents = json.loads(response)["distances"]
                
    if transport_mode == "N1":
      for index in xrange(len(locations)):
        contents[0][index] = 0

    if transport_mode == "1N":
      for index in xrange(len(locations)):
          contents[index][0] = 0
    dist_matrix = contents
  else:
    for from_node in xrange(len(locations)):
      dist_matrix[from_node] = {}
      for to_node in xrange(len(locations)):
        if (from_node == to_node) or (transport_mode == "1N" and to_node == 0) or (transport_mode == "N1" and from_node == 0):
          dist_matrix[from_node][to_node] = 0
        else:
          distance = (vincenty_distance(
            locations[from_node],
            locations[to_node]))
          dist_matrix[from_node][to_node] = distance
  return dist_matrix
def create_distance_callback(dist_matrix):
  # Create the distance callback.

  def distance_callback(from_node, to_node):
    return int(dist_matrix[from_node][to_node])

  return distance_callback

def tsp(event, context):
  # Create the data.
  try:
    locations = event["points"]
    transport_mode = event["transport_mode"]
    distance_calculation = event.get("distance_calculation", "VINCENTY")

  # Error handling
  except KeyError as e:
    print("Missing required input: " + str(e))
    cluster = {"title": "Missing required input: " + str(e)}
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

  dist_matrix = create_distance_matrix(locations, transport_mode, distance_calculation)
  dist_callback = create_distance_callback(dist_matrix)
  tsp_size = len(locations)
  num_routes = 1
  depot = 0

  # Create routing model.
  if tsp_size > 0:
    routing = pywrapcp.RoutingModel(tsp_size, num_routes, depot)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:

      # Solution cost.
      print("Total distance: " + str(assignment.ObjectiveValue()) + "\n")

      # Inspect solution.
      # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1.
      route_number = 0
      node = routing.Start(route_number)
      if transport_mode == "N1":
        node = assignment.Value(routing.NextVar(node)) 
      start_node = node
      route = ''
      cluster = []

      while not routing.IsEnd(node):
        cluster.append(locations[node])
        route += str(node) + ' -> '
        node = assignment.Value(routing.NextVar(node))
      
      if transport_mode != "1N":
        route += '0'
        cluster.append([locations[0]])
      print("Route:\n\n" + route)

      return return_lambda_gateway_response(200, {"route": cluster})
    else:
      print('No solution found.')
  else:
    print('Specify an instance greater than 0.')

def main():
  event = test_20
  print(tsp(event, ""))

if __name__ == '__main__':
  main()