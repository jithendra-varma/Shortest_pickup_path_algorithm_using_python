from __future__ import division
from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

"""Conditions"""

# Let us imagine that we are a startup pickup company. 
# We will have a set of pickup destinations, made known to us prior to the day of pickup, 
# So as to plan a route that starts from our pickup depot, goes to each destination and ends back at the same pickup depot. 
# This route will be optimised to either be of shortest distance or shortest time duration required. 
# We also only have one pickup van. As such, we are only looking at a basic TSP and not the Vehicle Routing Problem.

"""Address grouping based on pincode."""

# Inputs:
# 1) Input file containing the addresses as csv file.
# 2) Input the pickup depot address.
# 3) Input the minimum no of orders to be taken per pincode to deploy a pickup agent.

# This algorithm works only when certain minimum no of orders have reached at a particular pincode.
# Works collectively for any no of pincodes.

import gmplot
import json
import urllib.request as ur
import datetime
import csv

"""Part 1"""
# Taking the input values from csv file and converting them according to our need.

def address_list(filename,origin_city,min_orders):
    with open(filename,'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        # Group the locations based on pincodes and updating them onto a dictionary.
        next(csv_reader)
        pincode_dct = dict()
        for line in csv_reader:
            if line[4] not in pincode_dct:
                pincode_dct[line[4]] = []
            else:
                pincode_dct[line[4]].append(line[3])
        # Checking the condition of minimum orders from a location is met
        # Appending all the locations satisfying the condition on to location_list
        location_list = [origin_city]
        for k, v in pincode_dct.items():
            if len(v) >= min_orders:
                for i in v:
                    # Concatenating the pincode with address for better results.
                    st = i+" "+str(k)
                    location_list.append(st)
        return location_list

def receive_input(locations):
    # This function takes the input locations from the address_list function.
    # This function replaces the space separated strings(" ") into plus sign ("+") seperated strings.
    # Such that addresses can be used with google maps.

    addresses = []
    labels = locations
    num = 1
    for address in locations:
        address = address.replace(" ", "+")
        addresses.append(address)
    return addresses, labels

"""Part 2"""
# Taking the address values and generating the distance and duration matrix using google distance matrix api.

def create_distance_duration_matrix(addresses, API_key):
    # Distance Matrix API only accepts 100 elements per request, so we get rows in multiple requests.
    max_elements = 100
    num_addresses = len(addresses) # Let us take example 12
    # Maximum number of rows that can be computed per request (8 in this example).
    max_rows = max_elements // num_addresses
    # num_addresses = q * max_rows + r (q = 1 and r = 4 in this example).
    q, r = divmod(num_addresses, max_rows)
    dest_addresses = addresses
    distance_matrix = []
    duration_matrix = []
    # Send q requests, returning max_rows rows per request.
    for i in range(q):
        origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
        response = send_request(origin_addresses, dest_addresses, API_key)
        distance_matrix += build_distance_matrix(response)
        duration_matrix += build_duration_matrix(response)
    
    # Get the remaining remaining r rows, if necessary.
    if r > 0:
        origin_addresses = addresses[q * max_rows: q * max_rows + r]
        response = send_request(origin_addresses, dest_addresses, API_key)
        distance_matrix += build_distance_matrix(response)
        duration_matrix += build_duration_matrix(response)
    return distance_matrix, duration_matrix

def send_request(origin_addresses, dest_addresses, API_key):
    # Building and sending request for the given origin and destination addresses.
    def build_address_str(addresses):
        # Build a pipe-separated string of addresses
        address_str = ''
        for i in range(len(addresses) - 1):
            address_str += addresses[i] + '|'
        address_str += addresses[-1]
        return address_str

    request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=imperial'
    origin_address_str = build_address_str(origin_addresses)
    dest_address_str = build_address_str(dest_addresses)
    request = request + '&origins=' + origin_address_str + '&destinations=' + \
        dest_address_str + '&key=' + API_key
    # Urllib package is the URL handling module for python.
    # This ur.urlopen(request).read() opens and reads the URL.
    jsonResult = ur.urlopen(request).read()
    # .loads() function converts json to python output.
    response = json.loads(jsonResult)
    return response

def build_distance_matrix(response):
    # We are adding the row of size len(addresses) per each request. 
    distance_matrix = []
    for row in response['rows']:
        row_list = [row['elements'][j]['distance']['value'] for j in range(len(row['elements']))]
        distance_matrix.append(row_list)
    return distance_matrix

def build_duration_matrix(response):
    # We are adding the row of size len(addresses) per each request. 
    duration_matrix = []
    for row in response['rows']:
        row_list = [row['elements'][j]['duration']['value'] for j in range(len(row['elements']))]
        duration_matrix.append(row_list)
    return duration_matrix

"""Part 3"""

def create_data_model(distance_matrix, duration_matrix):
    # stores the data for the problem.
    data = {}
    data['distance_matrix'] = distance_matrix
    data['duration_matrix'] = duration_matrix
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def print_distance_solution(manager, distance_routing, duration_routing, solution, labels):
    # prints solution on console.
    # using google ortools to find the optimizied distance path.
    index = distance_routing.Start(0)
    plan_output = '\nRoute based on shortest distance:\n'
    route_distance = 0
    route_duration = 0
    Distance_ordered_list = []
    while not distance_routing.IsEnd(index):
        plan_output += ' {} ->'.format(labels[manager.IndexToNode(index)])
        Distance_ordered_list.append(labels[manager.IndexToNode(index)])
        previous_index = index
        index = solution.Value(distance_routing.NextVar(index))
        route_distance += distance_routing.GetArcCostForVehicle(previous_index, index, 0)
        route_duration += duration_routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(labels[manager.IndexToNode(index)])
    plan_output += 'Route distance: {}km\n'.format(route_distance/1000)
    plan_output += 'Route duration: {}\n'.format(datetime.timedelta(seconds=route_duration))
    print(plan_output)
    return Distance_ordered_list

def print_duration_solution(manager, duration_routing, distance_routing, solution, labels):
    # prints solution on console.
    # using google ortools to find the optimizied duration path.
    index = duration_routing.Start(0)
    plan_output = 'Route based on shortest duration:\n'
    route_duration = 0
    route_distance = 0
    Duration_ordered_list = []
    while not duration_routing.IsEnd(index):
        plan_output += ' {} ->'.format(labels[manager.IndexToNode(index)])
        Duration_ordered_list.append(labels[manager.IndexToNode(index)])
        previous_index = index
        index = solution.Value(duration_routing.NextVar(index))
        route_duration += duration_routing.GetArcCostForVehicle(previous_index, index, 0)
        route_distance += distance_routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(labels[manager.IndexToNode(index)])
    plan_output += 'Route distance: {}km\n'.format(route_distance/1000)
    plan_output += 'Route duration: {}'.format(datetime.timedelta(seconds=route_duration))
    print(plan_output)
    return Duration_ordered_list

"""Part 4"""

def Distance_latlong_gen(address_list,apikey):
    # converting the string address into latitude and latitude values.
    Dist_latlong_list = []
    for i in address_list:
        location = gmplot.GoogleMapPlotter.geocode(i, apikey=apikey)
        Dist_latlong_list.append(location)
    return Dist_latlong_list

def Distance_mapplot(latlong_list,apikey):
    # Plotting the path of the pickup
    lat,long = latlong_list[0]
    gmap = gmplot.GoogleMapPlotter(lat, long, 13, apikey=apikey, title="Smallest Distance Pickup Path")
    gmap.directions(
                (lat,long),
                (lat,long),
                waypoints=latlong_list[1:]
            )
    # Plotting the markers on the map
    attractions = zip(*latlong_list[1:])
    color_list = []
    for i in range(len(latlong_list[1:])):
        color_list.append('#87CEEB')
    marker_list = []
    for i in range(len(latlong_list[1:])):
        marker_list.append(True)
    label_list = []
    for i in range(len(latlong_list[1:])):
        label_list.append(str(i+1))
    
    gmap.scatter(
        *attractions,
        color=color_list,
        s=60,
        ew=2,
        marker=marker_list,
        label=label_list,
    )
    # Saving the map as a HTML document.
    gmap.draw('Pickup(distance)map.html')

def Duration_latlong_gen(address_list,apikey):
    # converting the string address into latitude and latitude values.
    Dura_latlong_list = []
    for i in address_list:
        location = gmplot.GoogleMapPlotter.geocode(i, apikey=apikey)
        Dura_latlong_list.append(location)
    return Dura_latlong_list

def Duration_mapplot(latlong_list,apikey):
    # Plotting the path of the pickup
    lat,long = latlong_list[0]
    gmap = gmplot.GoogleMapPlotter(lat, long, 13, apikey=apikey, title="Smallest Duration Pickup Path")
    gmap.directions(
                (lat,long),
                (lat,long),
                waypoints=latlong_list[1:]
            )
    # Plotting the markers on the map
    attractions = zip(*latlong_list[1:])
    color_list = []
    for i in range(len(latlong_list[1:])):
        color_list.append('#87CEEB')
    marker_list = []
    for i in range(len(latlong_list[1:])):
        marker_list.append(True)
    label_list = []
    for i in range(len(latlong_list[1:])):
        label_list.append(str(i+1))
    gmap.scatter(
        *attractions,
        color=color_list,
        s=60,
        ew=2,
        marker=marker_list,
        label=label_list,
    )
    # Saving the map as a HTML document.
    gmap.draw('Pickup(duration)map.html')
    

"""Part 5"""
# The main function of the program

if __name__ == '__main__':

    """Inputs"""
    input_file_name = input("Enter the file name of the csv file with extension .csv : ")
    origin_city = input("Enter the pickup depot location : ")
    min_orders = int(input("Enter the min_orders : "))
    API_Key = "" # Enter your api key here.
    
    # Part 1
    locations = address_list(input_file_name,origin_city,min_orders)
    addresses, labels = receive_input(locations)

    # Part 2
    distance_matrix, duration_matrix = create_distance_duration_matrix(addresses,API_Key)

    # Part 3
    # Instantiate the data problem
    data = create_data_model(distance_matrix, duration_matrix)

    ##### DISTANCE #####
    # Create the distance routing index manager.
    distance_manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    # Create Distance Routing Model.
    distance_routing = pywrapcp.RoutingModel(distance_manager)
    
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = distance_manager.IndexToNode(from_index)
        to_node = distance_manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    distance_transit_callback_index = distance_routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    distance_routing.SetArcCostEvaluatorOfAllVehicles(distance_transit_callback_index)

    ##### Duration #####
    # Create the distance routing index manager.
    duration_manager = pywrapcp.RoutingIndexManager(len(data['duration_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Distance Routing Model.
    duration_routing = pywrapcp.RoutingModel(duration_manager)

    def duration_callback(from_index, to_index):
        """Returns the time between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = duration_manager.IndexToNode(from_index)
        to_node = duration_manager.IndexToNode(to_index)
        return data['duration_matrix'][from_node][to_node]

    duration_transit_callback_index = duration_routing.RegisterTransitCallback(duration_callback)

    # Define cost of each arc.
    duration_routing.SetArcCostEvaluatorOfAllVehicles(duration_transit_callback_index)

    ##########

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    distance_solution = distance_routing.SolveWithParameters(search_parameters)
    duration_solution = duration_routing.SolveWithParameters(search_parameters)

    # Print solution on console and generating maps.
    if distance_solution:
        Distance_ordered_list = print_distance_solution(distance_manager, distance_routing, duration_routing, distance_solution, labels)
        Dist_latlonglist = Distance_latlong_gen(Distance_ordered_list,API_Key)
        Distance_mapplot(Dist_latlonglist,API_Key)
    if duration_solution:
        Duration_ordered_list = print_duration_solution(duration_manager, duration_routing, distance_routing, duration_solution, labels)
        Dura_latlonglist = Duration_latlong_gen(Duration_ordered_list,API_Key)
        Duration_mapplot(Dura_latlonglist,API_Key)
