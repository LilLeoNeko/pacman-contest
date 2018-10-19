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
import random, time, util, sys, math
from game import Directions
import game
from util import nearestPoint

#################
# Team creation #
#################


def createTeam(firstIndex,
               secondIndex,
               isRed,
               first='OffensiveMCTAgent',
               second='DefensiveMCTAgent'):
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
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


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
        closedFood = (0, 0)
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

        openList = util.Queue()  # [((x,y), Direction, cost)]
        closedList = []
        path = util.Queue()  # [All path the agent goes through]
        start = gameState.getAgentPosition(self.index)  # (x, y)

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

                for newState, action in [(self.getSuccessor(curState, action),
                                          action)
                                         for action in curLegalActions]:
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

        CONST_ITERATION_TIMES = 30
        CONST_MONTE_CARLO_DEPTH = 5

        start = time.time()
        mct, actionList = self.Monte_Carlo_Tree(gameState, iter_times=CONST_ITERATION_TIMES,\
                 simulate_depth=CONST_MONTE_CARLO_DEPTH)
        print 'eval time for agent %d: %.4f' % (self.index,
                                                time.time() - start)

        childs = mct.childs
        values = [child.reward / child.cnt for child in childs]
        max_value = max(values)
        nextState = [c for c, v in zip(childs, values) if v == max_value][0]

        #return nextState.gameState.getAgentState(self.index).configuration.direction
        nextStateIndex = mct.childs.index(nextState)
        curAction = gameState.getAgentState(self.index).configuration.direction
        nextAction = actionList[nextStateIndex]
        if curAction == Directions.REVERSE[nextAction]:
            print 'REVERSE!!!!!!!!!!!!!!!!!!!!!'
            return random.choice(actionList)
        return nextAction

    # construct a monte carlo tree depending on the current gameState
    def Monte_Carlo_Tree(self, gameState, iter_times, simulate_depth):
        root = Node(gameState, 0.0, 0)

        # root does not need to be simulated
        # so add the child directly
        #successors = self.generateSuccessors(root.gameState)
        successors, sucActionList = self.getAllCrossPoints(root.gameState)
        for suc in successors:
            child = Node(suc, 0.0, 0)
            root.addChild(child)

        for i in range(iter_times):
            curNode = root
            # part1 selection
            childs = curNode.childs
            while childs is not None:
                uct_values = [
                    self.uct(child, child.parent) for child in childs
                ]
                max_uct_values = max(uct_values)
                curNode = [
                    c for c, v in zip(childs, uct_values)
                    if v == max_uct_values
                ][0]
                childs = curNode.childs
            # curNode's child is None, should select a random action from this node

            # part2 expansion
            # if this node's cnt is 0, this node should be simulated
            # intead of expanded
            if curNode.cnt != 0:
                #successors = self.generateSuccessors(curNode.gameState)
                successors, _ = self.getAllCrossPoints(curNode.gameState)
                for suc in successors:
                    child = Node(suc, 0.0, 0)
                    child.printNode()
                    curNode.addChild(child)
                curNode = curNode.childs[0]  # random pick a child to simulate

            # part3 simulation
            reward = self.simulate(curNode.gameState, simulate_depth, 0.0)
            # print "reward: ", reward
            # print "curNode: ", curNode.printNode()
            # print "curNode's parentNode"

            # part4 backpropagate
            while curNode.parent is not None:
                curNode.reward += reward
                curNode.cnt += 1
                # print "what happen!!!?: ", curNode.reward
                curNode = curNode.parent
            # root node also need to be updated
            root.reward += reward
            root.cnt += 1

        #     print "root: ", root.printNode()
        #     print "root's child: ", root.printChilds()

        # print "last root: ", root.printNode()
        # print "last root's child: ", root.printChilds()
        return root, sucActionList

    def getNextCrossPoint(self, gameState, step):
        curAction = gameState.getAgentState(self.index).configuration.direction
        rev = Directions.REVERSE[curAction]
        legalActions = gameState.getLegalActions(self.index)
        usefulActions = list(set(legalActions) - set([Directions.STOP, rev]))

        if len(usefulActions) < 2:
            if len(usefulActions) == 1:
                nextAction = usefulActions[0]
                suc = self.getSuccessor(gameState, nextAction)
                if nextAction != curAction:
                    return suc
                return self.getNextCrossPoint(suc, step - 1)
            else:
                return None

        return gameState

    def getAllCrossPoints(self, gameState, step=20):
        curAction = gameState.getAgentState(self.index).configuration.direction
        sucList = self.generateSuccessors(gameState)
        cpList = []
        cpActionList = []
        cp = None
        for suc in sucList:
            cp = self.getNextCrossPoint(suc, step)
            if cp is not None and cp not in cpList:
                cpList.append(cp)
                cpActionList.append(suc.getAgentState(self.index).configuration.direction)

        return cpList, cpActionList

    def generateSuccessors(self, gameState):
        '''
    generate all successors with legal actions except STOP on the
    current state
    '''
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        print "actions: ", actions
        return [self.getSuccessor(gameState, action) for action in actions]

    # Input a successorState selected by UCT of last gameState
    # return the reward
    def simulate(self, gameState, level, reward):
        # Get the initial action and next action
        rev = Directions.REVERSE[gameState.getAgentState(
            self.index).configuration.direction]
        legalActions = gameState.getLegalActions(self.index)
        legalActions.remove(Directions.STOP)
        nextAction = random.choice(legalActions)
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
            if op.getPosition() is None:
                continue
            if not op.isPacman:
                opX, opY = op.getPosition()
                if (int(opX), int(opY)) == sucPos:
                    return curReward - 999

        if levelLeft > 0:
            return self.simulate(suc, levelLeft, curReward)
        return curReward

    # def uct(self, gameState, reward, count, prevCount):
    #     import math

    #     if prevCount == 0:
    #       return 0

    #     Cp = 0.5
    #     if count > 0:
    #         ucb = 2 * Cp * math.sqrt(2 * math.log(prevCount) / count)
    #     else:
    #         ucb = 99999999999
    #     return reward + ucb

    def uct(self, node, parentNode):
        Cp = 0.5
        cnt = node.cnt

        if cnt > 0:
            ucb = 2 * Cp * math.sqrt(2 * math.log(parentNode.cnt) / cnt)
        else:
            ucb = 999999999999

        return node.reward + ucb

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
        features['successorScore'] = -len(foodList)  #self.getScore(successor)

        rev = Directions.REVERSE[gameState.getAgentState(
            self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        # Compute distance to the nearest food
        myPos = successor.getAgentState(self.index).getPosition()
        if len(
                foodList
        ) > 0:  # This should always be True,  but better safe than sorry
            minDistance = min(
                [self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        features['distanceToOp'] = 0
        op_indeics = self.getOpponents(gameState)
        for op in op_indeics:
            if not successor.getAgentState(op).isPacman:
                opPos = successor.getAgentPosition(op)
                if opPos is not None:
                    features['distanceToOp'] += self.getMazeDistance(
                        myPos, opPos)

        return features

    def getWeights(self, gameState, action):
        return {'successorScore': 100, 'distanceToFood': -1, 'reverse': -10, 'distanceToOp': -150}


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
        enemies = [
            successor.getAgentState(i) for i in self.getOpponents(successor)
        ]
        invaders = [
            a for a in enemies if a.isPacman and a.getPosition() != None
        ]
        features['numInvaders'] = len(invaders)
        if len(invaders) > 0:
            dists = [
                self.getMazeDistance(myPos, a.getPosition()) for a in invaders
            ]
            features['invaderDistance'] = min(dists)

        if action == Directions.STOP: features['stop'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(
            self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        return features

    def getWeights(self, gameState, action):
        return {
            'numInvaders': -1000,
            'onDefense': 100,
            'invaderDistance': -10,
            'stop': -100,
            'reverse': -2
        }


class Node:
    """
    A node in a monte_carlo_tree of pacman using in this
    project.
    """

    def __init__(self, gameState, reward, cnt, parent=None, childs=None):
        self.gameState = gameState
        self.reward = reward
        self.cnt = cnt
        self.parent = parent
        self.childs = childs  # childs is a list of node

    def addChild(self, c):
        if self.childs is None:
            self.childs = [c]
        else:
            self.childs += [c]
        c.parent = self

    def printNode(self):
        print "Monte Carlo!!!!!!!!"
        #print self.gameState
        print "reward: ", self.reward, " cnt: ", self.cnt

    def printChilds(self):
        for child in self.childs:
            child.printNode()
