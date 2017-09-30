# A message that contains an uncompressed n dimensional
# matrix. Designed to be compatible with the opencv n-dimensional
# matrix.
Header header

int32[] sizes # The size of each dimension in the matrix

string encoding # The data type see src/image_encodings.cpp

bool is_bigendian # Is the data bigendian?

uint8[] data # The actual data
