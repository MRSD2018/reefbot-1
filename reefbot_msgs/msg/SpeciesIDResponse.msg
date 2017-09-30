# Response to a SpeciesIDRequest that tells the user the best guess of
# what species were found.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: June 2010

Header header

# Optional Id to identify the image
uint64 image_id

# Answers, one for each region in the Request and in the same order.
SingleSpeciesId[] answers
