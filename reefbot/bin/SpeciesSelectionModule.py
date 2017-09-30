'''Controls the user selection of which species the fish is.

When a new ImageCaptured message arrives, this module resets.

When the corresponding SpeciesIDResponse arrives, the module decides
whether the score was high enough. If it is, then the best species id
is sent to the UI.

If the score is not high enough, this module interacts directly
between the joystick messages and the user interface. It listens to
button presses on the joystick in order to change the currently
selected species. The species can be chosen using the arrow buttons on
the joystick and then the user\'s selection is made by pressing one of
the buttons. At that point, a ROS message is created (for logging
purposes) that tags the image with a species id and info about that
species is sent to the UI.

Author: Mark Desnoyer (markd@cmu.edu)
Date: Sept 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import ImageCaptured
from reefbot_msgs.msg import SpeciesIDResponse
from reefbot_msgs.msg import UserSpeciesSelection
from joy.msg import Joy
import RobotXMLStatus
from ImagePath import ImagePath
import math
import MySQLdb

def speciesIdCallback(data, module):
  """Handler for the SpeciesIDResponse messages from ROS."""
  module.ReceiveNewSpeciesIDResponse(data)

def imageCallback(data, module):
  """Handler for the ImageCaptured messages from ROS."""
  module.ReceiveNewImage(data)

def buttonCallback(data, module):
  """Handler for the Joystick messages from ROS."""
  module.ReceiveButtonEvent(data)

class State:
  DOING_NOTHING = 0
  IMAGE_ARRIVED = 1
  CANDIDATES_ARRIVED = 2
  USER_SELECTING = 3
  SPECIES_INFO_DISPLAYING = 4

class SpeciesInfo:
  """Specifies information about a single species that could be displayed."""
  def __init__(self, speciesId, commonName, latinName, imageFilename, earthImage, blurbs, maxSize, maxWeight, maxAge):
    self.id = speciesId
    self.name = commonName
    self.latin_name = latinName
    self.img_fn = imageFilename
    self.earth_img = earthImage
    self.blurbs = blurbs
    self.max_size = maxSize
    self.max_weight = maxWeight
    self.max_age = maxAge

  def GetOutputDict(self):
    return self.__dict__

  def __cmp__(self, other):
    self.id.__cmp__(other.id)


UNKNOWN_SPECIES = SpeciesInfo(-1, 'Species Not Listed', None,
                              ['images/UnknownFishSilhouette.png'],
                              None, None, None, None, None)

class SpeciesSelectionModule(RobotXMLStatus.Module): 
  def __init__(self, threadSignal):
    RobotXMLStatus.Module.__init__(self, threadSignal)
    self.state = State.DOING_NOTHING
    self.LoadSpeciesInfo()
    rospy.Subscriber(rospy.get_param("joystick_topic", "joy"),
                     Joy, buttonCallback, self)
    rospy.Subscriber(rospy.get_param('species_id_response_topic',
                                     'species_id'),
                     SpeciesIDResponse, speciesIdCallback, self)
    rospy.Subscriber(rospy.get_param('still_image_topic', 'still_image'),
                     ImageCaptured, imageCallback, self)
    self.publisher = rospy.Publisher(rospy.get_param('species_selected_topic',
                                                     'species_selected'),
                                     UserSpeciesSelection, latch=True)
    self.minScore = rospy.get_param('~min_species_score', -1)
    self.confidentScore = rospy.get_param('~confident_score', 1.0)
    self.changeAxis = rospy.get_param('~change_selection_axis', 5)
    self.decButton = rospy.get_param('~select_decrement_button', 0)
    self.incButton = rospy.get_param('~select_increment_button', 3)
    self.selButton = rospy.get_param('~select_species_button', 2)
    self.imageId = None
    self.pathGenerator = ImagePath()
    self.curIdx = 0
    self.userChose = False
    self.buttonState = {}
    self.buttonState[self.selButton] = False
    self.buttonState[self.incButton] = False
    self.buttonState[self.decButton] = False
    self.axisState = [False, False]
    self.speciesHits = None
    self.bbox = None

  def LoadSpeciesInfo(self):
    """Loads all the species information from the database."""
    self.speciesInfo = {}
    rospy.loginfo("Loading the list of species from the database.")
    
    db = MySQLdb.connect(
      host=rospy.get_param("mysql_host", "localhost"),
      user=rospy.get_param("mysql_ro_user", "reefbot_readonly"),
      passwd=rospy.get_param("mysql_pass", "starfish"),
      db=rospy.get_param("mysql_db","reefbot_console"))
    cursor = db.cursor()

    try:
      # Get the list of all the species
      cursor.execute("select id, common_name, sci_name, max_size, "
                     "max_weight, max_age from species where in_tank = True;")
      for (id, name, latinName, maxSize, maxWeight, maxAge) in \
              cursor.fetchall():
        info = SpeciesInfo(id, name, latinName, None, None, None,
                           maxSize, maxWeight, maxAge)

        # Find all the blurbs for this species
        cursor.execute("select fact from species_facts "
                       "where species_id = %i;" % id);
        info.blurbs = [x[0] for x in cursor.fetchall()]

        # Find all the images of this fish
        cursor.execute("select filename from species_pictures where "
                       "species_id = %i;" % id)
        info.img_fn = [x[0] for x in cursor.fetchall()]

        # See if we have an image of the species range on the earth
        cursor.execute("select filename from species_range where "
                       "species_id = %i;" % id);
        result = cursor.fetchone()
        if result is not None:
          info.earth_img = result[0]

        self.speciesInfo[id] = info;
    finally:
      cursor.close()

    rospy.loginfo("Found %d different species" % len(self.speciesInfo))

  def ReceiveNewImage(self, data):
    self.imageId = data.image_id
    self.state = State.IMAGE_ARRIVED
    self.SignalUpdate()

  def ReceiveNewSpeciesIDResponse(self, data):
    '''A new species has been identified.'''
    if data.image_id <> self.imageId:
      # We got a response for an image that's not the current one
      return

    # Reset the variables for selecting the species
    self.userChose = False
    self.curIdx = 0
    self.speciesHits = None
    self.state = State.CANDIDATES_ARRIVED
    self.bbox = None

    if len(data.answers) == 0:
      # There were no known species returned, so display unknown to the user
      self.speciesHits = [UNKNOWN_SPECIES.GetOutputDict()]
      self.state = State.SPECIES_INFO_DISPLAYING
      self.SignalUpdate()
      return

    # Use the bounding box and results from the first region
    subImageData = data.answers[0]
    
    if (subImageData.bounding_box.height > 0 and
        subImageData.bounding_box.width > 0):
      self.bbox = {}
      for attrib in subImageData.bounding_box.__slots__:
        self.bbox[attrib] = getattr(subImageData.bounding_box, attrib)
      
    if len(subImageData.best_species) > 0:
      # See if the score is above the confidence threshold. If so, the
      # user doesn't select the species.
      if subImageData.best_species[0].score >= self.confidentScore:
        try:
          self.speciesHits = [self.speciesInfo[
            subImageData.best_species[0].species_id].GetOutputDict()]
        except KeyError:
          rospy.logerr(("Species Id: %d does not exist but it was in the "
                          "SpeciesIDResponse") %
                       subImageData.best_species[0].species_id)

      # There wasn't a strong preference, so load up the species that
      # we'll ask the user about.
      else:
        self.speciesHits = []
        for species in subImageData.best_species:
          if species.score >= self.minScore:
            try:
              info = self.speciesInfo[species.species_id]
              self.speciesHits.append(info.GetOutputDict())
            except KeyError:
              rospy.logerr(("Species Id: %d does not exist but it was in the "
                            "SpeciesIDResponse") % species.species_id)

      if len(self.speciesHits) == 1:
        # There's only one species, so skip allowing the user to decide
        self.state = State.SPECIES_INFO_DISPLAYING
      else:
        self.speciesHits.append(UNKNOWN_SPECIES.GetOutputDict())

    else:
      # There were no known species returned, so display unknown to the user
      self.speciesHits = [UNKNOWN_SPECIES.GetOutputDict()]
      self.state = State.SPECIES_INFO_DISPLAYING

    self.SignalUpdate()

  def ReceiveButtonEvent(self, joystickMsg):
    '''Handles the joystick button presses to update the currently selected species.'''
    if self.state <> State.USER_SELECTING:
      return
    
    if self.NewButtonPress(joystickMsg, self.decButton):
      # Decrement the selection
      self.curIdx = (self.curIdx - 1) % len(self.speciesHits)
      self.SignalUpdate()
    if self.NewButtonPress(joystickMsg, self.incButton):
      # Increment the selection
      self.curIdx = (self.curIdx + 1) % len(self.speciesHits)
      self.SignalUpdate()
    if self.NewButtonPress(joystickMsg, self.selButton):
      # User selected the the species, so create a message to tag the image
      self.userChose = True
      self.speciesHits = [self.speciesHits[self.curIdx]]
      self.state = State.SPECIES_INFO_DISPLAYING
      if self.speciesHits[0]['id'] <> UNKNOWN_SPECIES.id:
        self.publisher.publish(species_id=self.speciesHits[0]['id'],
                               image_id=self.imageId,
                               image_path=self.pathGenerator.GetOSImagePath(
                                 self.imageId))
      else:
        self.state = State.DOING_NOTHING
      self.SignalUpdate()

  def NewAxisChange(self, joystickMsg, axis, desiredVal, stateIdx):
    '''Returns true if the message says that the axis has just gotten to desiredVal.'''
    atDesiredVal = math.fabs(joystickMsg.axes[axis] - desiredVal) < 1e-4;
    if self.axisState[stateIdx]:
      # We were already at the desired value
      self.axisState[stateIdx] = atDesiredVal;
      return False
    self.axisState[stateIdx] = atDesiredVal;

    return atDesiredVal;

  def NewButtonPress(self, joystickMsg, buttonId):
    '''Returns true if the message says that buttonId has been newly pressed.'''
    buttonIsPressed = joystickMsg.buttons[buttonId] != 0
    if self.buttonState[buttonId]:
      # Button was already pressed
      self.buttonState[buttonId] = buttonIsPressed;
      return False
    self.buttonState[buttonId] = buttonIsPressed;

    return buttonIsPressed

  def GetOutputDict(self):
    retval = {'name': 'SpeciesSelection', 'state': self.state}
    if self.state == State.IMAGE_ARRIVED:
      # A new image has arrived, so just send that path along
      retval['image_path'] =  self.pathGenerator.GetUIImagePath(self.imageId);

      
    elif self.state == State.CANDIDATES_ARRIVED or \
             self.state == State.USER_SELECTING:
      # Send data about the candidates
      retval['image_path'] = self.pathGenerator.GetUIImagePath(
        self.imageId);
      if self.bbox is not None:
        retval['bbox'] = self.bbox
      if self.speciesHits is None or len(self.speciesHits) <= 1:
        rospy.logerr("Invalid list of species found %s" % self.speciesHits)
      else:
        retval['species_hits'] = self.speciesHits
                                                  
      self.state = State.USER_SELECTING
        
      # Send data about the current selection made by the user
      retval['idx'] = self.curIdx
      retval['user_chose'] = self.userChose

      
    elif self.state == State.SPECIES_INFO_DISPLAYING:
      # Send data about the species that should be displayed to the user
      retval['species'] = self.speciesHits[0]
      if self.bbox is not None:
        retval['bbox'] = self.bbox
      retval['image_path'] = self.pathGenerator.GetUIImagePath(self.imageId);
        
      # Did the user actually choose the species    
      retval['user_chose'] = self.userChose

    return retval
