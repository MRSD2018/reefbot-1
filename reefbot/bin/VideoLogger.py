#!/usr/bin/python
'''Small utility that runs vlc to create video logs as we go along

Author: Mark Desnoyer(markd@cmu.edu)
Date: Oct 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import LogVideo
import datetime
import os.path
import vlc
import threading

def StartVideoLogging(msg, logger):
  logger.StartLogging(msg.rtp_ip.data, msg.rtp_port.data, msg.duration)

def RestartVideoLogging(logger):
  logger.RestartLogging()

class VideoLogger:
  def __init__(self, destDir, maxDuration):
    self.destDir = destDir
    self.maxDuration = maxDuration

    self.curSrc = None
    self.lock = threading.Semaphore(1)
    self.timer = None
    self.durationLeft = 0
    self.player = vlc.Instance().media_player_new()

  def StartLogging(self, ip, port, duration):
    '''Starts logging the video found at ip address and port

    ip - Ip address of the video broadcast
    port - Port of the video broadcast
    duration - length of video to log. If 0, goes forever until this is
    called again.
    '''
    if self.curSrc == (ip, port):
      # We're already logging this stream
      return
    if duration < 0:
      self.durationLeft = float("inf")
    else:
      self.durationLeft = duration

    self._StartLoggingImpl(ip, port, self.durationLeft)

  def _GetDestinationDir(self):
    day = datetime.date.today().strftime("%Y%m%d")
    return '%s/%s/' % (destDir, day)

  def RestartLogging(self):
    self._StartLoggingImpl(self.curSrc[0], self.curSrc[1], self.durationLeft)

  def _StartLoggingImpl(self, ip, port, duration):

    self.lock.acquire()

    # Close down the old log process
    self.player.stop()

    if duration <= 0:
      return

    self.curSrc = (ip, port)

    # Make the logging directory
    destDir = self._GetDestinationDir()
    if not os.path.exists(destDir):
      try:
        os.makedirs(destDir)
      except OSError as e:
        rospy.logerr("Could not create directory for logs: %s" % str(e))
        raise e

    # Start a new one
    fileName = datetime.datetime.now().strftime("%H-%M-%S") + '.mp4'
    destFile = os.path.join(destDir, fileName)
    rospy.loginfo("Logging video: " + destFile)

    self.player.set_mrl(("rtp://@%s:%s" % self.curSrc),
                        'sout=#file{dst=%s}' % destFile,
                        'rtp-caching=1200',
                        'sout-mux-caching=1200')
    self.player.play()

    # Set a time to stop the log file
    logDuration = duration
    if logDuration <= 0 or duration > self.maxDuration:
      logDuration = self.maxDuration
    self.timer = threading.Timer(logDuration, RestartVideoLogging, [self])
    self.timer.start()
    rospy.logwarn("This log will continue for %d seconds" % logDuration)
    self.durationLeft = self.durationLeft - logDuration

    self.lock.release()

  def StopLogging(self):
    self.lock.acquire()
    self._StopLoggingImpl()
    self.lock.release()

  
  def _StopLoggingImpl(self):
    '''Assumes that a lock has been made on this object.'''
    if self.timer is not None:
      self.timer.cancel()
      self.timer = None

    if self.loggingProc is not None:
      rospy.loginfo("Closing down the video log")
      retCode = self.loggingProc.poll()
      if retCode is not None:
        rospy.logerr("The logging process terminated unexpectedly. "
                     "See the logs where the video is located.")
      else:
        self.loggingProc.terminate()
        retCode = self.loggingProc.wait()
        if retCode > 0:
          rospy.logerr("There was a problem with the video logger.")

      self.loggingProc = None
      self.vlcLog.close()
      self.vlcLog = None
      self.curURL = None

if __name__ == '__main__':
  rospy.init_node('VideoLogger')

  # Directory for log videos
  destDir = rospy.get_param("~dest_dir", "/export/reefbot/video/")

  # Maximum video duration in seconds
  maxDuration = rospy.get_param("~max_duration", 600)

  videoLogger = VideoLogger(destDir, maxDuration)

  rospy.Subscriber(rospy.get_param('log_video_topic', 'log_video'),
                   LogVideo,
                   StartVideoLogging,
                   videoLogger)

  rospy.spin()
  
  
