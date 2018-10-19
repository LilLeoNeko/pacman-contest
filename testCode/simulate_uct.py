# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'OffensiveMCTAgent', second = 'DefensiveMCTAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex)]#, eval(second)(secondIndex)]

##########
# Agents #
##########
offensiveInitPath = []
class MCTCaptureAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """
  # def __init__(self):
  #   self.offensiveInitPath = []



  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """
    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''
    # self.start = gameState.getAgentPosistion(self.index)
    global offensiveInitPath
    foodList = self.getFood(gameState).asList()
    myPos = gameState.getAgentState(self.index).getPosition()
    closedFood = (0,0)
    minDist = 9999
    d = 0
    for food in foodList:
      d = self.getMazeDistance(myPos, food)
      if d < minDist:
        closedFood = food
        minDist = d

    offensiveInitPath = self.bfs(gameState, closedFood)
    print "after bfs: " + str(offensiveInitPath)


  def bfs(self, gameState, goal):
    """Search the shallowest nodes in the search tree first."""

    curState = gameState

    openList = util.Queue() # [((x,y), Direction, cost)]
    closedList = []
    path = util.Queue() # [All path the agent goes through]
    start = gameState.getAgentPosition(self.index) # (x, y)

    # openList.push((start, "Start", 0))
    openList.push(curState)
    path.push([])


    while not openList.isEmpty():
        curState = openList.pop()
        curPos = curState.getAgentPosition(self.index)
        # if problem.isGoalState(state[0]):
        print "state: " + str(curPos) + " goal: " + str(goal)
        if curPos == goal:
            print "arrive goal: " + str(path.pop())
            return list(reversed(path.pop()))

        prePath = path.pop()

        # if state[0] not in closedList:
        if curState not in closedList:

            # for newstate in problem.getSuccessors(state[0]):
            # for newstate in gameState.getSuccessors(state):
            curLegalActions = [action  \
                          for action in curState.getLegalActions(self.index) \
                          if action != Directions.STOP]

            for newState, action in [(self.getSuccessor(curState, action), action) for action in curLegalActions]:
              #curPos = newState.getAgentPosition(self.index)
              if newState not in closedList:
                openList.push(newState)
                curPath = prePath[:]
                curPath.append(action)
                path.push(curPath)

            closedList.append(curState)

    return []

  def chooseAction(self, gameState):
    '''
    You should change this in your own agent.
    '''
    # global offensiveInitPath
    # print "aaaaaaaaaaa: " + str(offensiveInitPath)
    # if len(offensiveInitPath) > 1:
    #   return offensiveInitPath.pop()


    foodLeft = len(self.getFood(gameState).asList())

    # if already got 5 food, go back home.
    if foodLeft <= 5:
      return self.goBackHome(gameState)

    actions = gameState.getLegalActions(self.index)

    # generate 15 possible 3-move actions
    CONST_CNT = 5
    north = 0
    south = 0
    west = 0
    east = 0
    actions = { Directions.NORTH: 0, Directions.SOUTH: 0, \
                Directions.WEST: 0, Directions.EAST: 0 } #list of actions

    start = time.time()

    for x in range(50):
      # start action is None and start reward is 0
      action, reward = self.simulate(gameState, CONST_CNT, None, 0)
      print "action and reward after a simulation: ", action, reward
      actions[action] += reward

      if action == Directions.NORTH: north += 1
      if action == Directions.SOUTH: south += 1
      if action == Directions.WEST: west += 1
      if action == Directions.EAST: east += 1

    print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    # start = time.time()
    # # monte carlo, using simulation to evaluate the value of each actions
    # values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    # maxValue = max(values)
    # bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    for action, value in actions.items():
      if north != 0 and action == Directions.NORTH: actions[action] /= north
      if south != 0 and action == Directions.SOUTH: actions[action] /= south
      if west != 0 and action == Directions.WEST: actions[action] /= west
      if east != 0 and action == Directions.EAST: actions[action] /= east

    action_sorted_keys = sorted(actions, key=actions.get, reverse=True)

    i = 0
    while action_sorted_keys[i] not in gameState.getLegalActions(self.index):
      i += 1
    return action_sorted_keys[i]

  # Input a successorState selected by UCT of last gameState
  # return the reward
  def simulate(self, gameState, level, reward):
        # Get the initial action and next action
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        legalActions = gameState.getLegalActions(self.index)
        nextAction = random.choice(set(legalActions) - set([Directions.STOP, rev]))

        '''
        while nextAction == Directions.STOP or nextAction == rev:
            nextAction = random.choice(gameState.getLegalActions(self.index))
        '''
        suc = self.getSuccessor(gameState, nextAction)
        sucPos = suc.getAgentPosition(self.index)
        levelLeft = level - 1
        foods = self.getFood(gameState).asList()
        curReward = reward

        curReward += self.evaluate(gameState, nextAction)
        '''
        # If successor is a food, reward +1
        if sucPos in foods:
            #import pdb; pdb.set_trace()
            curReward += 100

        # If successor is a Ghost, reward -100
        op_indices = self.getOpponents(gameState)
        for op in op_indices:
            if not suc.getAgentState(op).isPacman:
                if sucPos == suc.getAgentPosition(op):
                    curReward -= 9999
        '''
        opStates = [suc.getAgentState(i) for i in self.getOpponents(suc)]
        for op in opStates:
            if op is None:
                continue
            if not op.isPacman:
                opX, opY = op.getPosition()
                if (int(opX),int(opY)) == sucPos:
                    return curReward - 99

        if levelLeft > 0:
            return self.simulate(suc, levelLeft, curReward)
        return curReward

  def uct(self, gameState, reward, count, prevCount):
      import math
      Cp = 0.5
      if count > 0:
          ucb = 2 * Cp * math.sqrt(2 * math.log(prevCount) / count)
      else:
          ucb = 99999999999
      return reward + ucb

  def goBackHome(self, gameState):
    actions = gameState.getLegalActions(self.index)
    bestDist = 9999
    for action in actions:
      successor = self.getSuccessor(gameState, action)
        pos2 = successor.getAgentPosition(self.index)
        dist = self.getMazeDistance(self.start,pos2)
        opp1, opp2 = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        opp1Pos = opp1.getPosition()
        opp2Pos = opp2.getPosition()
        if pos2 == opp1Pos or pos2 == opp2Pos:
          dist = dist + 9999
        if not opp1Pos == None and self.getMazeDistance(pos2,opp1Pos) == 1:
          dist = dist + 9999
        if not opp2Pos == None and self.getMazeDistance(pos2,opp2Pos) == 1:
          dist = dist + 9999
        if dist < bestDist:
          bestAction = action
          bestDist = dist
    return bestAction


  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()

    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)
    return features * weights

  def getFeatures(self, gameState, action):
    """
    Returns a counter of features for the state
    """
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    features['successorScore'] = self.getScore(successor)
    return features

  def getWeights(self, gameState, action):
    """
    Normally, weights do not depend on the gamestate.  They can be either
    a counter or a dictionary.
    """
    return {'successorScore': 1.0}

class OffensiveMCTAgent(MCTCaptureAgent):
  """
  A reflex agent that seeks food. This is an agent
  we give you to get an idea of what an offensive agent might look like,
  but it is by no means the best or only way to build an offensive agent.
  """
  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    foodList = self.getFood(successor).asList()
    features['successorScore'] = -len(foodList)#self.getScore(successor)

    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    # Compute distance to the nearest food
    myPos = successor.getAgentState(self.index).getPosition()
    if len(foodList) > 0: # This should always be True,  but better safe than sorry
      minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
      features['distanceToFood'] = minDistance

    op_indeics = self.getOpponents(gameState)
    for op in op_indeics:
      if not successor.getAgentState(op).isPacman:
        opPos = successor.getAgentPosition(op)
        features['distanceToOp'] += self.getMazeDistance(myPos, opPos)

    return features

  def getWeights(self, gameState, action):
    return {'successorScore': 100, 'distanceToFood': -1, 'reverse': -10}

class DefensiveMCTAgent(MCTCaptureAgent):
  """
  A reflex agent that keeps its side Pacman-free. Again,
  this is to give you an idea of what a defensive agent
  could be like.  It is not the best or only way to make
  such an agent.
  """

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    if myState.isPacman: features['onDefense'] = 0

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)
    if len(invaders) > 0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
      features['invaderDistance'] = min(dists)

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}
