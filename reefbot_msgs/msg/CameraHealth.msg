# Message that specifies the camera's health
#
# Author: Mark Desnoyer
# Date: July 2010

Header header

# Can we ping the camera?
bool ping_ok

# Can we connect to the http server on the camera
bool http_ping_ok

# Error code reported by the camera. 0 means that there is no
# error. See CameraWatchdog.py for error codes.
int32 error_code
