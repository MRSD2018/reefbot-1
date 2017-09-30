// Abstract class for a set of image descriptors like SIFT or SURF features.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: June 2010

#ifndef _SPECIES_ID_IMAGE_DESCRIPTOR_INL_H__
#define _SPECIES_ID_IMAGE_DESCRIPTOR_INL_H__

#include <vector>
#include "ImageDescriptor.h"

using namespace std;

namespace species_id {

template <typename T>
ostream& ImageDescriptorINRIASerializer<T>::Write(
  ostream& stream,
  const ImageDescriptor<T>& descriptor) const {
  stream << descriptor.point().x << ' '
         << descriptor.point().y << ' '
         << "0 0 0 "; // The a, b, c

  // Now write the descriptor values
  for (int i=0; i < descriptor.DescriptorSize(); ++i) {
    stream << descriptor.GetVal(i) << ' ';
  }
  return stream;
}

template <typename T>
ostream& ImageDescriptorINRIASerializer<T>::WriteHeader(
  ostream& stream,
  const vector<typename ImageDescriptor<T>::Ptr>& descriptors) const {
  if (descriptors.size() == 0) {
    return WriteHeader(stream, 128, 0);
  }

  return WriteHeader(stream, descriptors[0]->DescriptorSize(),
                     descriptors.size());
}

template <typename T>
ostream& ImageDescriptorINRIASerializer<T>::Write(
  ostream& stream,
  const vector<typename ImageDescriptor<T>::Ptr>& descriptors) const {
  
  // First write the header
  WriteHeader(stream, descriptors);

  // Now write the descriptors for each keypoint
  for (typename vector<
         typename ImageDescriptor<T>::Ptr>::const_iterator i = 
         descriptors.begin();
       i != descriptors.end(); ++i) {
    Write(stream, **i) << endl;
  }
  stream.flush();

  return stream;
}

}

#endif //_SPECIES_ID_IMAGE_DESCRIPTOR_INL_H__
