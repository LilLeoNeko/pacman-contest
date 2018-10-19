# -*- coding: UTF-8 -*-
# baselineTeam.py
# ---------------
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


# baselineTeam.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

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
               first = 'OffensiveReflexAgent', second = 'DefensiveReflexAgent'):
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
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):
  """
  A base class for reflex agents that chooses score-maximizing actions
  """
 
  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)


  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    actions = gameState.getLegalActions(self.index)

    # You can profile your evaluation time by uncommenting these lines
    # start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]

    foodLeft = len(self.getFood(gameState).asList())
    if foodLeft <= 2:
      bestDist = 9999
      for action in actions:
        successor = self.getSuccessor(gameState, action)
        pos2 = successor.getAgentPosition(self.index)
        dist = self.getMazeDistance(self.start,pos2)
        if dist < bestDist:
          bestAction = action
          bestDist = dist
      return bestAction

    return random.choice(bestActions)

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

class OffensiveReflexAgent(ReflexCaptureAgent):
  def getFeatures(self, gameState, action):
    features = util.Counter()
    if action == Directions.STOP: features['stop'] = 1

    succGameState = self.getSuccessor(gameState, action)  # successor is game state
    actions = succGameState.getLegalActions(self.index)
    succAgentState = succGameState.getAgentState(self.index)  # succState is agent state
    succPos = succAgentState.getPosition()
    enemies = [succGameState.getAgentState(i) for i in self.getOpponents(succGameState)]
    ghosts = [enemy for enemy in enemies if not enemy.isPacman and enemy.getPosition() != None]
    invaders = [enemy for enemy in enemies if enemy.isPacman and enemy.getPosition() != None]
    capsules = self.getCapsules(succGameState)
    # successor.get

    # get scores
    features['successorScore'] = self.getScore(succGameState)

    # eat opponent pacman
    if len(invaders) > 0 and not succAgentState.isPacman:
      dists = [self.getMazeDistance(succPos, invader.getPosition()) for invader in invaders]
      features['distanceToInvader'] = min(dists)

    # run away from ghost + reward -100 + -1000

    features['distanceToGhost'] = 99999
    if len(ghosts) > 0:
      minDistToGhost = min([self.getMazeDistance(succPos, ghost.getPosition()) for ghost in ghosts])
      nearestGhost = [ghost for ghost in ghosts if
                      self.getMazeDistance(succPos, ghost.getPosition()) == minDistToGhost]
      if nearestGhost[0].scaredTimer > 0:
        # print "scared: -----",nearestGhost[0].scaredTimer
        features['distanceToGhost'] = 99999
      elif succAgentState.isPacman:
        features['distanceToGhost'] = minDistToGhost
      elif not succAgentState.isPacman:
        features['nearToGhost'] = 1

      # if len(actions) <= 2 and len(ghosts) > 0:
      #     features['directions'] = len(actions)

    if len(actions) <= 3 and not succGameState.hasFood(int(succPos[0]), int(succPos[1])):
      features['drections'] = len(actions)

    # eat capsules
    features['remainCap'] = len(capsules)
    if len(capsules) > 0:
      features['distanceToCap'] = min([self.getMazeDistance(succPos, cap) for cap in capsules])

    # eat food +100
    foodList = self.getFood(succGameState).asList()
    features['remainFood'] = len(foodList)  # self.getScore(successor)
    # features['distanceToFood'] = 9999
    if len(foodList) > 0:  # This should always be True,  but better safe than sorry
      features['distanceToFood'] = min([self.getMazeDistance(succPos, food) for food in foodList])

    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    return features

  def getWeights(self, gameState, action):
    return {
      # 'successorScore': 1000,
      'remainFood': -100,
      'distanceToFood': -1,
      'remainCap': -100,
      'distanceToCap': -1,
      'distanceToGhost': 1000,
      'distanceToInvader': -200,
      'directions': -1000,  # 移动方向
      'stop': -3000,
      'nearToGhost': 10000,
      'reverse': -1000
    }
  # def getFeatures(self, gameState, action):
  #   features = util.Counter()
  #   successor = self.getSuccessor(gameState, action)
  #   foodList = self.getFood(successor).asList()
  #   features['successorScore'] = -len(foodList)#self.getScore(successor)
  #
  #   # Compute distance to the nearest food
  #
  #   if len(foodList) > 0: # This should always be True,  but better safe than sorry
  #     myPos = successor.getAgentState(self.index).getPosition()
  #     minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
  #     features['distanceToFood'] = minDistance
  #   return features
  #
  # def getWeights(self, gameState, action):
  #   return {'successorScore': 100, 'distanceToFood': -1}

class DefensiveReflexAgent(ReflexCaptureAgent):
  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.boundaryloc, self.searchActions = self.BFS(gameState)
    self.initFood = self.getFoodYouAreDefending(gameState).asList()
    # if red team
    if self.red:
      # print "RRRRRRRRed team"
      self.detectDest = [(self.boundaryloc[0]-1,self.boundaryloc[1])]
    else:
    #   print "BBBBBBBBule team"
      self.detectDest = [(self.boundaryloc[0]+1,self.boundaryloc[1])]


    self.chaseDest = []

  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    # import pdb; pdb.set_trace()
    print "------------------"

    actions = gameState.getLegalActions(self.index)

    # You can profile your evaluation time by uncommenting these lines
    # start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    currEnemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
    currInvader = [enemy for enemy in currEnemies if enemy.isPacman and enemy.getPosition() != None]
    foodList = self.getFoodYouAreDefending(gameState).asList()
    loc = gameState.getAgentState(self.index).getPosition()

    # randomDest = random.choice(foodList)
    # minDistToFood = min([self.getMazeDistance(loc, food) for food in foodList])
    # minDistFood = [food for food in foodList if self.getMazeDistance(loc, food) == minDistToFood]
    if len(self.initFood) - len(foodList) > 0:
      eatenFood = list(set(self.initFood).difference(set(foodList)))
      print
      print "food lost", eatenFood
      print
      self.initFood = foodList
      self.chaseDest = eatenFood

    if len(currInvader) > 0:
      foodLeft = len(self.getFood(gameState).asList())
      print "find invader"
      self.chaseDest = []          #####################################
        # print bestAction
        # return bestAction
    elif len(self.chaseDest)>0:
      print "my location", loc
      print "chase eaten food", self.chaseDest
      self.chaseFood = self.AStar(gameState,self.chaseDest[0]) #  chaseFood is several actions return by AStar
      if len(self.chaseFood)>0:          #  How to chase food 
        print self.chaseFood[0]
        return self.chaseFood[0]
      if loc == self.chaseDest[0]:       #  The location is the destination location(eaten food location)
        print "reach the eaten food destination"
        self.chaseDest = []
    #  if didn't detect any invader and didn't find any food has been eaten, then go to defend food
    elif len(self.detectDest)>0:
      print "my location", loc
      print "come to the detect destination", self.detectDest
      self.boundaryloc, self.searchActions = self.BFS(gameState)   #  The boudary location AND several actions to boudary location
      if len(self.searchActions)>0:        #  one of the actions to boudary location
        print self.searchActions[0]
        return self.searchActions[0]
      if loc == self.detectDest[0]:          #  arrive at destination, empty detectDest List
        self.detectDest=[]   
    else:
      self.boundaryloc, self.searchActions = self.BFS(gameState)
      if self.red:
        self.detectDest = [(self.boundaryloc[0] - 1, self.boundaryloc[1])]
      else:
        self.detectDest = [(self.boundaryloc[0] + 1, self.boundaryloc[1])]
      if len(self.searchActions)>0:         #  one of the actions to boudary location
        print self.searchActions[0]
        return self.searchActions[0]

    return random.choice(bestActions)      #  choose best actions, if has same values, random choose one action as best action

  def BFS(self,gameState):
    from util import Queue
    visited, movements = [], {}
    startAgentState = gameState.getAgentState(self.index)
    startLoc = startAgentState.getPosition()

    movements[startLoc] = []
    visited.append(startLoc)

    queue = Queue()
    queue.push(gameState)

    while not queue.isEmpty():
      currGameState = queue.pop()
      currLoc = currGameState.getAgentState(self.index).getPosition()

      if self.isGoal(currGameState):
        return currLoc,movements[currLoc][:-1]

      actions = currGameState.getLegalActions(self.index)

      for action in actions:
        succGameState = self.getSuccessor(currGameState, action)
        succLoc = succGameState.getAgentState(self.index).getPosition()

        if succLoc not in visited:
          visited.append(succLoc)
          movements[succLoc] = []
          pre = movements[currLoc][:]
          pre.append(action)
          movements[succLoc].extend(pre)
          queue.push(succGameState)

  def AStar(self, gameState,destination):
    from util import PriorityQueue
    visited, movements, costs, start_cost = [], {}, {}, 0
    startAgentState = gameState.getAgentState(self.index)
    startLoc = startAgentState.getPosition()


    movements[startLoc] = []
    costs[startLoc] = 0
    visited.append(startLoc)

    priorityQueue = PriorityQueue()
    priorityQueue.push(gameState, start_cost)

    while not priorityQueue.isEmpty():
      currGameState = priorityQueue.pop()
      currLoc = currGameState.getAgentState(self.index).getPosition()

      if currLoc == destination:
        # print "movements: ",movements[currLoc]
        return movements[currLoc]
        # return movements[currGameState]

      actions = currGameState.getLegalActions(self.index)

      for action in actions:
        succGameState = self.getSuccessor(currGameState, action)
        succLoc = succGameState.getAgentState(self.index).getPosition()
        newCost = 1
        next_cost = costs[currLoc] + newCost

        if succLoc not in visited or next_cost<costs[succLoc]:
          visited.append(succLoc)
          movements[succLoc] = []
          
          pre = movements[currLoc][:]
          pre.append(action)
          movements[succLoc].extend(pre)
          costs[succLoc] = next_cost
          heurisitic = self.Heuristic(succLoc,destination)
          priority = next_cost + heurisitic
          priorityQueue.push(succGameState,priority)

  def Heuristic(self,loc,destination):
    from util import manhattanDistance
    dist = manhattanDistance(loc,destination)
    # print "manhattanDistance: ", dist
    return dist

  def isGoal(self,gameState):
      agentState = gameState.getAgentState(self.index)
      if agentState.isPacman:
          return True
      else:
          return False


  def getFeatures(self, gameState, action):
    features = util.Counter()
    successorGameState = self.getSuccessor(gameState, action) # capture.py
    successorAgentState = successorGameState.getAgentState(self.index)
    myPos = successorAgentState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    if successorAgentState.isPacman: features['onDefense'] = 0
    if action == Directions.STOP: features['stop'] = 1


    # Computes distance to invaders we can see
    enemies = [successorGameState.getAgentState(i) for i in self.getOpponents(successorGameState)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)
    if len(invaders) > 0:
      minDist = min([self.getMazeDistance(myPos, a.getPosition()) for a in invaders])
      features['invaderDistance'] = minDist
      if successorAgentState.scaredTimer > 0 and minDist == 1:
        features['scared'] = 1
        features['stop'] = 0

    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000,
            'onDefense': 1000,
            'invaderDistance': -10,
            'stop': -100,
            'reverse': -100,
            'scared': -1000}