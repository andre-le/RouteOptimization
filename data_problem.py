###########################
# Problem Data Definition #
###########################
class DataProblem():
    """Stores the data for the problem"""
    def __init__(self, locations, num_vehicles, min_parcels, maximum_distance):
        """Initializes the data for the problem"""
        self._num_vehicles = num_vehicles
        self._maximum_distance = maximum_distance
        self._min_parcels = min_parcels
        self._locations = locations
        self._depot = 0
        self._parcels = [loc[2] for loc in locations]

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