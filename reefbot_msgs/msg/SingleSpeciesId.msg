# The best few matches for the species in a single region
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: June 2010

# The bounding box where we found the individual
sensor_msgs/RegionOfInterest bounding_box

# The most likely species in descending order. This array could be
# empty if there was no good match.
SpeciesScore[] best_species
