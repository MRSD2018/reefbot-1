# The score of a species identification. The score is dependent on the
# algorithm being used so it doesn't necessarily have semantic meaning
# except that a higher score is better.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: June 2010

# Unique identifier for the species
uint32 species_id

# Score for the species. Higher is better
float32 score

# Optional extra information about the score. This will be algorithm
# dependent and might be useful for debugging
string meta_data
