#!/usr/bin/python
'''Performs unittests for the SpeciesSelectionModule

Author: Mark Desnoyer (markd@cmu.edu)
Date: Oct 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import ImageCaptured
from reefbot_msgs.msg import SpeciesIDResponse
from reefbot_msgs.msg import UserSpeciesSelection
from reefbot_msgs.msg import SingleSpeciesId
from reefbot_msgs.msg import SpeciesScore
from sensor_msgs.msg import RegionOfInterest
from joy.msg import Joy
import unittest
import SpeciesSelectionModule
from SpeciesSelectionModule import SpeciesInfo, State
import RobotXMLStatus
from mock import Mock
from ImagePath import ImagePath

class TestStateLogic(unittest.TestCase):

  def LoadMockSpecies(self, moduleSelf):
    '''Function that will load a small number of species into the module.'''
    moduleSelf.speciesInfo = {}

    moduleSelf.speciesInfo[0] = SpeciesInfo(
      0,  # id
      'Blue Fish', # Name
      'traxis', # Sci Name
      ['blue_fish.jpg'], # img filename
      None, # earth image
      ['It is a blue fish', 'no really'], # blurbs
      10.0, # max_size
      2.0, # max_weight
      3) # max_age

    moduleSelf.speciesInfo[2] = SpeciesInfo(
      2,  # id
      'Red Fish', # Name
      'rouge', # Sci Name
      ['red_fish.jpg', 'red_fish2.jpg'], # img filename
      None, # earth image
      ['It is a red fish'], # blurbs
      8.1, # max_size
      2.3, # max_weight
      5.5) # max_age

    moduleSelf.speciesInfo[3] = SpeciesInfo(
      3,  # id
      'One Fish', # Name
      'uno', # Sci Name
      ['uno.jpg'], # img filename
      'earthUno.jpg', # earth image
      ['One fish, two fish', 'Red fish, blue fish', 'Yay!'], # blurbs
      18.1, # max_size
      2.4, # max_weight
      9) # max_age

    self.speciesInfo = moduleSelf.speciesInfo

  def ModuleInit(self, moduleSelf, threadSignal, publisher):
    '''New init function for the module that does not have dependencies.'''
    RobotXMLStatus.Module.__init__(moduleSelf, threadSignal)
    moduleSelf.state = State.DOING_NOTHING
    self.LoadMockSpecies(moduleSelf)

    moduleSelf.publisher = publisher

    moduleSelf.minScore = 0.1
    moduleSelf.confidentScore = 0.7
    moduleSelf.decButton = 1
    moduleSelf.incButton = 2
    moduleSelf.selButton = 4
    moduleSelf.imageId = None
    moduleSelf.pathGenerator = ImagePath('path/to/image_dir', 'html/path')
    self.pathGen = moduleSelf.pathGenerator
    moduleSelf.curIdx = 0
    moduleSelf.userChose = False
    moduleSelf.buttonState = {}
    moduleSelf.buttonState[moduleSelf.selButton] = False
    moduleSelf.buttonState[moduleSelf.decButton] = False
    moduleSelf.buttonState[moduleSelf.incButton] = False
    moduleSelf.speciesHits = None
    moduleSelf.bbox = None
    
    
  def setUp(self):
    # Modify the module class so that we aren't talking to an SQL server
    ModuleClass = SpeciesSelectionModule.SpeciesSelectionModule
    ModuleClass.__init__ = lambda x,y,z : TestStateLogic.ModuleInit(self, x, y, z)

    self.signalMock = Mock()
    self.pubMock = Mock()

    self.module = ModuleClass(self.signalMock, self.pubMock)

  def checkSignalUpdate(self, startIdx):
    self.signalMock.acquire.assert_called_with()
    self.signalMock.notifyAll.assert_called_with()
    self.signalMock.release.assert_called_with()

  def checkSpeciesInfo(self, fullObj, dictType):
    '''Checks that a SpeciesInfo object is represented the same in the dictionary that\'s returned.'''
    self.assertEquals(fullObj.id, dictType['id'])
    self.assertEquals(fullObj.name, dictType['name'])
    self.assertEquals(fullObj.latin_name, dictType['latin_name'])
    self.assertEquals(fullObj.img_fn, dictType['img_fn'])
    self.assertEquals(fullObj.earth_img, dictType['earth_img'])
    self.assertEquals(fullObj.blurbs, dictType['blurbs'])
    if fullObj.max_size is None:
      self.assertTrue(dictType['max_size'] is None)
    else:
      self.assertAlmostEqual(fullObj.max_size, dictType['max_size'])
    if fullObj.max_weight is None:
      self.assertTrue(dictType['max_weight'] is None)
    else:
      self.assertAlmostEqual(fullObj.max_weight, dictType['max_weight'])
    if fullObj.max_age is None:
      self.assertTrue(dictType['max_age'] is None)
    else:
      self.assertAlmostEqual(fullObj.max_age, dictType['max_age'])

  def test_newImage(self):
    SpeciesSelectionModule.imageCallback(ImageCaptured(3, None), self.module)

    self.checkSignalUpdate(0)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.IMAGE_ARRIVED)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertEquals(self.module.imageId, 3)

  def test_buttonPressBeforeImage(self):
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,4]), self.module)

    self.assertFalse(self.signalMock.called)

  def test_imageResponseBeforeImage(self):
    SpeciesSelectionModule.speciesIdCallback(SpeciesIDResponse(3, [
      SingleSpeciesId(None, [])]),
                                              self.module)

    self.assertFalse(self.signalMock.called)

class TestCandidateArrival(TestStateLogic):
  def setUp(self):
    TestStateLogic.setUp(self)

    # An initial image has already arrived
    SpeciesSelectionModule.imageCallback(ImageCaptured(3, None), self.module)

    self.signalMock.reset_mock()
    
  def test_differentImageId(self):
    SpeciesSelectionModule.speciesIdCallback(SpeciesIDResponse(4, [
      SingleSpeciesId(None, [])]),
                                              self.module)
    self.assertFalse(self.signalMock.called)

  def test_buttonPressBeforeSpeciesId(self):
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,4]), self.module)

    self.assertFalse(self.signalMock.called)

  def test_noBoundingBoxMultipleHits(self):
    species_id = SingleSpeciesId(None, [SpeciesScore(2, 0.4),
                                        SpeciesScore(3, 0.3)])

    SpeciesSelectionModule.speciesIdCallback(
      SpeciesIDResponse(3, [species_id]),
      self.module)

    self.checkSignalUpdate(3)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.CANDIDATES_ARRIVED)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertTrue('bbox' not in d.keys())
    self.assertEquals(len(d['species_hits']), 3)
    self.checkSpeciesInfo(self.speciesInfo[2], d['species_hits'][0])
    self.checkSpeciesInfo(self.speciesInfo[3], d['species_hits'][1])
    self.checkSpeciesInfo(SpeciesSelectionModule.UNKNOWN_SPECIES,
                          d['species_hits'][2])
    self.assertEquals(d['user_chose'], False)
    self.assertEquals(d['idx'], 0)

  def test_WithBoundingBox(self):
    bbox = RegionOfInterest(30, 40, 210, 302)
    species_id = SingleSpeciesId(bbox, [SpeciesScore(3, 0.4),
                                        SpeciesScore(2, 0.3)])

    SpeciesSelectionModule.speciesIdCallback(
      SpeciesIDResponse(3, [species_id]),
      self.module)

    self.checkSignalUpdate(3)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.CANDIDATES_ARRIVED)
    self.assertEquals(d['name'], 'SpeciesSelection')
    foundBbox = d['bbox']
    self.assertEquals(foundBbox['x_offset'], 30)
    self.assertEquals(foundBbox['y_offset'], 40)
    self.assertEquals(foundBbox['height'], 210)
    self.assertEquals(foundBbox['width'], 302)
    self.assertEquals(len(d['species_hits']), 3)
    self.checkSpeciesInfo(self.speciesInfo[3], d['species_hits'][0])
    self.checkSpeciesInfo(self.speciesInfo[2], d['species_hits'][1])
    self.checkSpeciesInfo(SpeciesSelectionModule.UNKNOWN_SPECIES,
                          d['species_hits'][2])
    self.assertEquals(d['user_chose'], False)
    self.assertEquals(d['idx'], 0)

  def test_singleHit(self):
    # Should go straight to displaying the species info
    species_id = SingleSpeciesId(None, [SpeciesScore(2, 0.4)])

    SpeciesSelectionModule.speciesIdCallback(
      SpeciesIDResponse(3, [species_id]),
      self.module)

    self.checkSignalUpdate(3)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.SPECIES_INFO_DISPLAYING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.checkSpeciesInfo(self.speciesInfo[2], d['species'])
    self.assertEquals(d['user_chose'], False)

  def test_confidentScore(self):
    # A score was high enough to be confident, so we should just
    # display the info
    species_id = SingleSpeciesId(None, [SpeciesScore(0, 0.8)])

    SpeciesSelectionModule.speciesIdCallback(
      SpeciesIDResponse(3, [species_id]),
      self.module)

    self.checkSignalUpdate(3)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.SPECIES_INFO_DISPLAYING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.checkSpeciesInfo(self.speciesInfo[0], d['species'])
    self.assertEquals(d['user_chose'], False)

  def test_onlyOneValidScore(self):
    # Only one of the scores is above the min threshold, so we display
    species_id = SingleSpeciesId(None, [SpeciesScore(0, 0.3),
                                        SpeciesScore(3, 0.05)])

    SpeciesSelectionModule.speciesIdCallback(
      SpeciesIDResponse(3, [species_id]),
      self.module)

    self.checkSignalUpdate(3)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.SPECIES_INFO_DISPLAYING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.checkSpeciesInfo(self.speciesInfo[0], d['species'])
    self.assertEquals(d['user_chose'], False)

class TestUserButtonSelection(TestCandidateArrival):
  def setUp(self):
    TestCandidateArrival.setUp(self)

    # An initial image has already arrived
    SpeciesSelectionModule.imageCallback(ImageCaptured(3, None), self.module)

    self.signalMock.reset_mock()

    # The ID was made with 3 results
    species_id = SingleSpeciesId(None, [SpeciesScore(0, 0.5),
                                        SpeciesScore(3, 0.4),
                                        SpeciesScore(2, 0.3)])

    SpeciesSelectionModule.speciesIdCallback(
      SpeciesIDResponse(3, [species_id]),
      self.module)
    # Make sure State.CANDIDATES_ARRIVED worked
    self.module.GetOutputDict()

    self.signalMock.reset_mock()

  def test_initialChoice(self):
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,0]), self.module)

    self.assertFalse(self.signalMock.called)

  def test_twoIncrMessagesOnePress(self):
    # Increment
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)

    self.checkSignalUpdate(3)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.USER_SELECTING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertEquals(d['idx'], 1)
    self.assertFalse(d['user_chose'])

    # Shouldn't trigger another message to the UI
    self.signalMock.reset_mock()
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)
    self.assertFalse(self.signalMock.called)

  def test_incrAroundHorn(self):
    # There are three entries and then UNKNOWN, so go around the horn

    for i in range(7):
      # Increment button press
      SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)
      # Increment button unpress
      SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,0]), self.module)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.USER_SELECTING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertEquals(d['idx'], 3)
    self.assertFalse(d['user_chose'])

  def test_decAroundHorn(self):
    # There are three entries and then UNKNOWN, so go around the horn

    for i in range(7):
      # Decrement button press
      SpeciesSelectionModule.buttonCallback(Joy([], [0,1,0,0,0]), self.module)
      # Decrement button unpress
      SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,0]), self.module)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.USER_SELECTING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertEquals(d['idx'], 1)
    self.assertFalse(d['user_chose'])

  def test_incAndDec(self):
    # There are three entries and then UNKNOWN, so go around the horn

    # Increment
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)
    # Decrement
    SpeciesSelectionModule.buttonCallback(Joy([], [0,1,0,0,0]), self.module)
    # Increment
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)

    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.USER_SELECTING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertEquals(d['idx'], 1)
    self.assertFalse(d['user_chose'])

  def test_choosingSpecies(self):
    # Increment
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,0]), self.module)
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,1,0,0]), self.module)

    # Press the choose button
    SpeciesSelectionModule.buttonCallback(Joy([], [0,0,0,0,1]), self.module)

    # Make sure the correct data is sent to the UI
    d = self.module.GetOutputDict()
    self.assertEquals(d['state'], State.SPECIES_INFO_DISPLAYING)
    self.assertEquals(d['name'], 'SpeciesSelection')
    self.assertTrue(d['user_chose'])
    self.checkSpeciesInfo(self.speciesInfo[2], d['species'])

    # Make sure that the choice is logged properly
    self.pubMock.publish.assert_called_with(species_id=2, image_id=3,
                                            image_path=self.pathGen.GetOSImagePath(3))

if __name__ == '__main__':
  unittest.main()
