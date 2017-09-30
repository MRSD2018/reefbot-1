#!/usr/bin/python
'''Chooses a random selection of species as the identificaiton. Scores are 0.

Input message: SpeciesIDRequest
Output message: SpeciesIDResponse

Author: Mark Desnoyer
Date: Sept 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import MySQLdb
import random
from reefbot_msgs.msg import SpeciesIDRequest
from reefbot_msgs.msg import SpeciesIDResponse
from reefbot_msgs.msg import SingleSpeciesId
from reefbot_msgs.msg import SpeciesScore

class RandomSpeciesSelector:
  def __init__(self):
    self.randGen = random.Random()
    self.randGen.seed(27638)
    self.LoadSpeciesList()

  def LoadSpeciesList(self):
    rospy.loginfo("Loading the list of species from the database.")
    db = MySQLdb.connect(
      host=rospy.get_param("mysql_host", "localhost"),
      user=rospy.get_param("mysql_ro_user", "reefbot_readonly"),
      passwd=rospy.get_param("mysql_pass", "starfish"),
      db=rospy.get_param("mysql_db","reefbot_console"))
    cursor = db.cursor()
    try:
      cursor.execute("SELECT id from species where in_tank = True;")
      self.speciesIds = [x[0] for x in cursor.fetchall()]
    finally:
      cursor.close()

  def ChooseNSpecies(self, n):
    if n >= len(self.speciesIds):
      return self.speciesIds
    
    return self.randGen.sample(self.speciesIds, n)

def RequestCallback(requestMsg, selector, publisher, nSpecies):
    '''Callback for a SpeciesIDRequest that chooses a random set of species.

    Inputs:
    selector - RandomSpeciesSelector to handle the random picks
    publisher - object to publish the response on
    nSpecies - number of species to respond with
    '''
    response = SpeciesIDResponse()
    response.image_id = requestMsg.image_id
    for region in requestMsg.regions:
      singleId = SingleSpeciesId()
      singleId.bounding_box = region.bounding_box
      for speciesId in selector.ChooseNSpecies(nSpecies):
        singleId.best_species.append(SpeciesScore(species_id=speciesId, score=0.0))
      response.answers.append(singleId)
    response.header.stamp = rospy.Time.now()
    publisher.publish(response)

if __name__ == '__main__':
  rospy.init_node('RandomSpeciesId')
  selector = RandomSpeciesSelector()

  publisher = rospy.Publisher(
    rospy.get_param('~species_id_response_topic', 'species_id'),
    SpeciesIDResponse,
    tcp_nodelay=True)
  callback = lambda x: RequestCallback(x,
                                       selector,
                                       publisher,
                                       rospy.get_param('~n_species', 4));
  rospy.Subscriber(rospy.get_param('~species_id_request_topic',
                                   'request_species_id'),
                   SpeciesIDRequest,
                   callback)

  rospy.spin()
