// C implementation of some detection utils for python. To make things fast
//
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Date: June 2012

#include <Python.h>

// Calculates the overlap between two tuples of the form (x,y,h,w)
// overlap = cDetectionUtils.CalculateOverlapOfRegions(boxA, boxB)
static PyObject* cDetectionUtils_CalculateOverlapOfRegions(
  PyObject *self,
  PyObject*args) {
  int xA, yA, hA, wA;
  int xB, yB, hB, wB;

  // Parse the function arguments
  if (!PyArg_ParseTuple(args, "(iiii)(iiii)", &xA, &yA, &hA, &wA,
                        &xB, &yB, &hB, &wB)) {
    return NULL;
  }

  if (hA < 0 || wA < 0 || hB < 0 || wB < 0) {
    return Py_BuildValue("d", 0.0);
  }
    
  double midYA = yA + hA/2.0;
  double midYB = yB + hB/2.0;
  double dRy = hA/2.0 + hB/2.0 - fabs(midYA - midYB);

  double midXA = xA + wA/2.0;
  double midXB = xB + wB/2.0;
  double dRx = wA/2.0 + wB/2.0 - fabs(midXA - midXB);

  if (dRx < 1e-5 || dRy < 1e-5) {
    // They do not overlap
    return Py_BuildValue("d", 0.0);
  }

  // They overlap so calculate the fraction overlap as the area that
  // overlaps over the area of the union.
  double areaIntersection = dRy * dRx;
  double areaUnion = ((double)hA) * wA + ((double)hB) * wB -
    areaIntersection;

  return Py_BuildValue("d", areaIntersection / areaUnion);
}

// List of functions exported by this module
static PyMethodDef cDetectionUtilsMethods[] = {
  {"CalculateOverlapOfRegions", cDetectionUtils_CalculateOverlapOfRegions,
   METH_VARARGS,
   "Calculates the overlap between two boxes in tuple form (x,y,h,w)"},
  {NULL, NULL, 0, NULL} // Sentinel
};

// Initialize the module
PyMODINIT_FUNC initcDetectionUtils(void) {
  Py_InitModule("cDetectionUtils", cDetectionUtilsMethods);
}
