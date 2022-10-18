# File for helper functions

# arduino map, from https://stackoverflow.com/questions/70643627/python-equivalent-for-arduinos-map-function
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
