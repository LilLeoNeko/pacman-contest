# 1. Simulate remove reverse, stop still here
# 2. 
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
    return [eval(first)(firstIndex), eval(second)(secondIndex)]
#def memberSwitch()
#def bothAttack()

##########
# Agents #
##########
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
        #self.start = gameState.getAgentPosistion(self.index)

    # def bfs(self, gameState, goal):
    #     """Search the shallowest nodes in the search tree first."""
        
    #     curState = gameState

    #     openList = util.Queue() # [((x,y), Direction, cost)]
    #     closedList = []
    #     path = util.Queue() # [All path the agent goes through]
    #     start = gameState.getAgentPosition(self.index) # (x, y)

    #     # openList.push((start, "Start", 0))
    #     openList.push(curState)
    #     path.push([])


    #     while not openList.isEmpty():
    #         curState = openList.pop()
    #         curPos = curState.getAgentPosition(self.index)
    #         # if problem.isGoalState(state[0]):
    #         ##print "state: " + str(curPos) + " goal: " + str(goal)
    #         if curPos == goal:
    #             ##print "arrive goal: " + str(path.pop())
    #             return list(reversed(path.pop()))

    #         prePath = path.pop()

    #         # if state[0] not in closedList:
    #         if curState not in closedList:

    #             # for newstate in problem.getSuccessors(state[0]):
    #             # for newstate in gameState.getSuccessors(state):
    #             curLegalActions = [action  \
    #                                 for action in curState.getLegalActions(self.index) \
    #                                 if action != Directions.STOP]
                                                                
    #             for newState, action in [(self.getSuccessor(curState, action), action) for action in curLegalActions]:
    #                 #curPos = newState.getAgentPosition(self.index)
    #                 if newState not in closedList:
    #                     openList.push(newState)
    #                     curPath = prePath[:]
    #                     curPath.append(action)
    #                     path.push(curPath)

    #             closedList.append(curState)
    #     return []

    def chooseAction(self, gameState):
        """
        Picks among actions randomly.
        """
        actions = gameState.getLegalActions(self.index)

        '''
        You should change this in your own agent.
        '''

        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()
        values = [self.evaluate(gameState, a) for a in actions]
        ### print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        return random.choice(actions)

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
    AttackAgent using Monte carlo tree to decide foods to eat and way to escape
    """

    def registerInitialState(self, gameState):
        
        CaptureAgent.registerInitialState(self, gameState)

        self.initialPosition = gameState.getInitialAgentPosition(self.index)
        ##print self.initialPosition,"!!!!!!!!!!!!!!!!!!!!!!!!!!"

        # Initialization calculate the homepoint
        # homepoint means the points along the middle Line of layout
        # Red on left, Blue on right, therefore blue needs + 1
        # pacman go to homepoint to score and turns into ghost
        self.homePoints = []

        if self.red:
            central = (gameState.data.layout.width - 2) / 2
        else:
            central = ((gameState.data.layout.width -2) / 2) + 1

        for height in range(1, gameState.data.layout.height-1):
            if not gameState.hasWall(central, height):
                self.homePoints.append((central, height))

        self.start = gameState.getAgentPosition(self.index)
        self.totalFood = len(self.getFood(gameState).asList())

        # Different Mode Flags

        self.survivalMode = False
        self.findCapMode = False
        self.powerMode = False

        # Entry point set up

        self.entryPoints = self.homePoints

        self.isOpRed = not self.red
        x,y = 0,0
        if self.isOpRed:
            x = 1
            y = 1
        else:
            x = gameState.data.layout.width - 2
            y = gameState.data.layout.height - 2
        ##print "!!!!!!!!!!!!!!!!", gameState.data.layout.width,"!!!!!!!!!!!!!!!!!!!"
        ##print "!!!!!!!!!!!!!!!!!!!",gameState.data.layout.height,"!!!!!!!!!!!!!!!!!!!"

        self.ghostPos = (x,y)

    def chooseAction(self, gameState):
        '''
        You should change this in your own agent.
        '''
        # Timer to use to test the performance
        start = time.time()

        # Get remain food as a list
        foodLeft = len(self.getFood(gameState).asList())
        
        # Iteration of Monte Carlo Tree
        # Simulation Depth of Monte Carlo
        CONST_ITERATION_TIMES = 50
        CONST_MONTE_CARLO_DEPTH = 10

        # Check 3 MODE of PacMan
        self.powerCheck(gameState)
        self.survivalCheck(gameState)
        self.findCapCheck(gameState)

        foodNum = gameState.getAgentState(self.index).numCarrying
        if (foodNum > (self.totalFood / 3) and not self.powerMode) or foodLeft < 2:
            self.findCapMode = False
            self.survivalMode = True

        if foodNum < 1:
            self.survivalMode = False

        # capusles = self.getCapsules(gameState)
        # if len(capusles) > 0 and not self.powerMode:
        #     myPos = gameState.getAgentPosition(self.index)
        #     minDToCap = min([self.getMazeDistance(myPos, cap) for cap in capusles])
        #     if minDToCap <= 10:
        #         self.findCapMode = True
        #         self.survivalMode = False 

        if not self.survivalMode and not self.powerMode \
            and not self.findCapMode and gameState.getAgentState(self.index).isPacman:
            MCT = self.MCT(gameState, iter_times=CONST_ITERATION_TIMES,\
                                    simulate_depth=CONST_MONTE_CARLO_DEPTH)

            childs = MCT.childs
            values = [child.reward/child.cnt for child in childs]
            max_value = max(values)
            nextNode = [c for c, v in zip(childs, values) if v == max_value][0]

            print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

            return nextNode.gameState.getAgentState(self.index).configuration.direction
        else:

            print "OMG OMG OMG OMG OMG OMG OMG"
            actions = gameState.getLegalActions(self.index)
            actions.remove(Directions.STOP)
            values = [self.evaluate(gameState, a) for a in actions]

            maxValue = max(values)
            bestActions = [a for a, v in zip(actions, values) if v == maxValue]
            print zip(actions, values)
            print "bestActions: ", bestActions
            if len(bestActions) > 0:
                return bestActions[0]
            print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

            return random.choice(actions)

    # turn on survival mode if ghost(s) chase you and no capsules left
    # turn off if no ghost is chasing you
    # turn off if you are no longer a pacman

    def survivalCheck(self, gameState):

        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        chaseGhosts = [a for a in enemies if (not a.isPacman) and a.getPosition() != None and a.scaredTimer == 0]
        #min([self.getMazeDistance(myState.getPosition(),cg.getPosition()) for cg in chaseGhosts])
        myState = gameState.getAgentState(self.index)
        foodNum = myState.numCarrying
        if len(chaseGhosts) > 0:
            gDist = min([self.getMazeDistance(myState.getPosition(),cg.getPosition()) for cg in chaseGhosts])

        if myState.isPacman:
            if len(chaseGhosts) > 0 and gDist <= 5 and not self.powerMode:
                self.survivalMode = True
        else:
            self.survivalMode = False

    # only when survival mode is not on, findCap mode can be activated
    def findCapCheck(self, gameState):
        myState = gameState.getAgentState(self.index)
        curCapsuleList = self.getCapsules(gameState)
        # if myState.isPacman and len(curCapsuleList) > 0:
        #     self.findCapMode = True
        if self.survivalMode and len(curCapsuleList) > 0 and not self.powerMode:
            self.findCapMode = True
            self.survivalMode = False
        else:
            self.findCapMode = False


    def powerCheck(self, gameState):
        myState = gameState.getAgentState(self.index)
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]

        minGhostScare = min(enemy.scaredTimer for enemy in enemies)
        if minGhostScare < 11:
            self.powerMode = False
        else:
            self.powerMode = True
            self.survivalMode = False

    # construct a monte carlo tree depending on the current gameState
    # eat MCT is used when survival mode is not activate

    def MCT(self, gameState, iter_times, simulate_depth):

        startTime = time.time()


        root = Node(gameState, 0.0, 0)
        # root does not need to be simulated
        # so add the child directly
        successors = self.generateSuccessors(root.gameState)
        for suc in successors:
            child = Node(suc, 0.0, 0)
            root.addChild(child)

        for i in range(iter_times):
            if time.time() - startTime > 0.95:
                break
            curNode = root
            # part1 selection
            childs = curNode.childs
            while childs is not None:
                    uct_values = [self.uct(child, child.parent) for child in childs]
                    max_uct_values = max(uct_values)
                    curNode = [c for c, v in zip(childs, uct_values) if v == max_uct_values][0]
                    childs = curNode.childs
            # curNode's child is None, should select a random action from this node

            # part2 expansion
            # if this node's cnt is 0, this node should be simulated
            # intead of expanded
            if curNode.cnt != 0:
                successors = self.generateSuccessors(curNode.gameState)
                for suc in successors:
                    child = Node(suc, 0.0, 0)
                    #child.##printNode()
                    curNode.addChild(child)
                curNode = curNode.childs[0] # random pick a child to simulate

            # part3 simulation
            reward = self.simulate(curNode.gameState, simulate_depth, 0.0)

            # part4 backpropagate
            while curNode.parent is not None:
                curNode.reward += reward
                curNode.cnt += 1
                # ##print "what happen!!!?: ", curNode.reward
                curNode = curNode.parent
            # root node also need to be updated
            root.reward += reward
            root.cnt += 1

        return root

    def generateSuccessors(self, gameState, mode="Normal"):
        '''
        generate all successors with legal actions except STOP on the
        current state
        '''
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        ###print "actions: ", actions
        return [self.getSuccessor(gameState, action) for action in actions]

    # Input a successorState selected by UCT of last gameState
    # return the reward
    def simulate(self, gameState, level, reward):
        # Get the initial action and next action
        legalActions = gameState.getLegalActions(self.index)

        # if legal action have more than two actions(except stop and rev), remove rev
        # to ensure better simulation performance
        if len(legalActions) > 2:
            rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
            legalActions.remove(Directions.STOP)
            if rev is not Directions.STOP:
                legalActions.remove(rev)

        nextAction = random.choice(legalActions)

        suc = self.getSuccessor(gameState, nextAction)
        sucPos = suc.getAgentPosition(self.index)
        levelLeft = level - 1

        curReward = reward
        curReward += self.evaluate(gameState, nextAction)

        # update the best_entry to go to eat food
        # BEST_ENTRY
        if not suc.getAgentState(self.index).isPacman:
            dtoGhost = 99999.9
            opStates = [suc.getAgentState(i) for i in self.getOpponents(suc)]
            for op in opStates:
                if op.getPosition() is None:
                    continue
                if not op.isPacman:
                    d = self.getMazeDistance(sucPos, op.getPosition())
                    if d < dtoGhost:
                        dtoGhost = d
                        self.ghostPos = op.getPosition()
        # BEST_ENTRY END

        # regard eating capusles as a terminate state
        if self.findCapMode and sucPos in self.getCapsules(gameState):
            return curReward

        # if dead
        if suc.getAgentPosition is self.start and not self.powerMode:
            return curReward - 999999

        if levelLeft > 0:
            return self.simulate(suc, levelLeft, curReward)
        return curReward

    def uct(self, node, parentNode):
            import math
            
            Cp = 0.75
            cnt = node.cnt

            if cnt > 0:
                    ucb = 2 * Cp * math.sqrt(2 * math.log(parentNode.cnt) / cnt)
            else:
                    ucb = 999999999999

            return node.reward + ucb

    # def getFeatures(self, gameState, action):
    #     features = util.Counter()
    #     successor = self.getSuccessor(gameState, action)
    #     myState = successor.getAgentState(self.index)
    #     foodList = self.getFood(successor).asList()    
    #     features['successorScore'] = -len(foodList)#self.getScore(successor)

    #     rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    #     if action == rev: features['reverse'] = 1

    #     # Compute distance to the nearest food
    #     myPos = myState.getPosition()
    #     if len(foodList) > 0: # This should always be True,  but better safe than sorry
    #         minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
    #         features['distanceToFood'] = minDistance

    #     # Compute distance to the closest observable enemies

    #     op_indeics = self.getOpponents(gameState)
    #     if op_indeics is None:
    #         pass
    #     else:
    #         minOpDistance = 9999
    #         for op in op_indeics:
    #             if not successor.getAgentState(op).isPacman:
    #                 opPos = successor.getAgentPosition(op)
    #                 if opPos is not None:
    #                     opDistance = self.getMazeDistance(myPos, opPos)
    #                     if opDistance < minOpDistance:
    #                         minOpDistance = opDistance
    #         if minOpDistance != 9999:
    #             features['distanceToOp'] = minOpDistance
            

    #     # for op in op_indeics:
    #     #     if not successor.getAgentState(op).isPacman:
    #     #         opPos = successor.getAgentPosition(op)
    #     #         if opPos is not None:
    #     #             if self.getMazeDistance(opPos, myPos) is 1:
    #     #                 features['distanceToOp'] = -100
    #     #             else:
    #     #                 features['distanceToOp'] = self.getMazeDistance(myPos, opPos)

    #     # BEST_ENTRY

    #     if not gameState.getAgentState(self.index).isPacman:
    #         bestEntry = (0,0)
    #         d_ghostToEntry = 0.0
    #         for ep in self.entryPoints:
    #             d = self.getMazeDistance(self.ghostPos,ep)
    #             if d > d_ghostToEntry:
    #                 d_ghostToEntry = d
    #                 bestEntry = ep
    #         features['distanceToEntry'] = self.getMazeDistance(myPos,bestEntry)
    #     # BEST_ENTRY END
                
    #     # Compute distance to the closest home point
    #     if self.survivalMode is True:
    #         minHomeDist = min([self.getMazeDistance(myPos,hp) for hp in self.homePoints])
    #         features['distanceToHome'] = minHomeDist

    #     # when a pacman arrived home safely
    #     if self.survivalMode is True and myPos in self.homePoints:
    #         features['arrivedHome'] = self.getScore(gameState)

    #         if myPos is self.initialPosition:
    #             features['dead'] = 1

    #     # possibleActions = successor.getLegalActions(self.index)
    #     # if len(possibleActions) <= 1:
    #     #     features['deadEnd'] = 1

    #     if self.findCapMode is True:
    #         capsules = self.getCapsules(gameState)
    #         if len(capsules) != 0:
    #             minDistance = min([self.getMazeDistance(myPos, capPos) \
    #                                 for capPos in capsules])
    #             features['distanceToCap'] = minDistance

    #     return features

    def getFeatures(self, gameState, action):

        features = util.Counter()

        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        foodList = self.getFood(successor).asList()
        myPos = myState.getPosition()
        isPacman = gameState.getAgentState(self.index).isPacman

        # we hope it had better not stop
        if action is Directions.STOP:
            features['STOP'] = 1

        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        # Compute distance to the nearest food
        if len(foodList) > 0: # This should always be True,  but better safe than sorry
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        features['successorScore'] = -len(foodList)

        # Compute distance to the closest ghost
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        chaseGhosts = [a for a in enemies if (not a.isPacman) and a.getPosition() != None and a.scaredTimer <= 10]
        if len(chaseGhosts) > 0:
            minGhostDistance = self.dToNearestOp(successor)
            features['distanceToGhost'] = minGhostDistance

        # compute the number of ghost in distance 1
        for cg in chaseGhosts:
            cgDis = self.dToNearestOp(successor)
            if cgDis <= 1:
                features['chaseGhosts'] = 1

        # # Refuse the conrners!
        # """ when I am a pacman """
        # if gameState.getAgentState(self.index).isPacman:
        #     actions = successor.getLegalActions(self.index)
        #     if actions <= 2: features['deadEnd'] = 1

        """ when I am a ghost """ 
        if not isPacman:

            # find a best Entry if current Entry being stucked by ghost
            bestEntry = (0,0)
            d_ghostToEntry = 0.0
            for ep in self.entryPoints:
                d = self.getMazeDistance(self.ghostPos,ep)
                if d > d_ghostToEntry:
                    d_ghostToEntry = d
                    bestEntry = ep
            features['distanceToEntry'] = self.getMazeDistance(myPos, bestEntry)

            # got little reward if kill the op
            minPacDistance = self.dToNearestOp(successor, isPacman)
            if minPacDistance is not None:
                features['distanceToPacman'] = minPacDistance

            if successor.getAgentState(self.index).isPacman:
                features['isPacman'] = 1

            # got little reward if kill the op
            minPacDistance = self.dToNearestOp(successor, isPacman)
            if minPacDistance is not None:
                features['distanceToPacman'] = minPacDistance

        """ Power Mode """
        if self.powerMode:

            capsules = self.getCapsules(successor)
            if len(capsules) != 0:
                minDistance = min([self.getMazeDistance(myPos, capPos) \
                                    for capPos in capsules])
                features['distanceToCap'] = minDistance

            return features

        """ survival Mode """
        if self.survivalMode:
        # Compute distance to the closest home point
        # minHomeDist = min([self.getMazeDistance(myPos,hp) for hp in self.homePoints])
            """ we can go to the place that we miss food """
            dToHome = self.getMazeDistance(myPos, (1,1))
            if dToHome == 0:
                features['distanceToHome'] = 9999
            else:
                features['distanceToHome'] = dToHome

            if myPos is self.initialPosition:
                features['dead'] = 1

            if not successor.getAgentState(self.index).isPacman and myPos is not self.start:
                features['arrivedHome'] = 1

            return features          

        """ Find Cap Mode """
        if self.findCapMode:
            #Compute distance to the closest capsules point
            capsules = self.getCapsules(successor)
            if len(capsules) != 0:
                minDistance = min([self.getMazeDistance(myPos, capPos) \
                                    for capPos in capsules])
                features['distanceToCap'] = minDistance
            features['eatenCap'] = -len(capsules)

        return features

    def getWeights(self, gameState, action):

        if self.powerMode:
            print "*******Using Power Mode********"
            return {'successorScore': 150, 'distanceToFood': -1, 'distanceToCap': 5, 'chaseGhosts': -5}

        if self.survivalMode:
            print "*******Using survival Mode********"
            return {'distanceToHome': -1, 'chaseGhosts': -99999999, 'dead': -999999999}

        if self.findCapMode:
            print "*******Using Find caps Mode********"
            return {'distanceToCap': -1,'eatenCap': 150, 'chaseGhosts': -500}

        print "*******Using Normal Mode********"
        return {'successorScore': 150, 'distanceToFood': -5, 'reverse': -2,\
                'distanceToEntry': -5, 'STOP': -5, 'isPacman': 150, 'chaseGhosts': -500}

        # # if power mode is on, eat the food greedy until its nearly off (10 steps left)
        # if self.powerMode is True:
        #     print "*******Using Power Mode********"
        #     return {'successorScore': 150, 'distanceToFood': -10}

        # # if survival mode is on, encourage our offensive agent to go back home
        # if self.findCapMode is True:
        #     print "*******Using Find caps Mode********"
        #     return {'distanceToCap': -200, 'dead':-9999,'successorScore': 150, 'distanceToFood': -5, 'reverse': -3,'distanceToEntry': -5}
        
        # if self.survivalMode is True:
        #     print "*******Using survival Mode********"
        #     return {'dead': -9999, 'arrivedHome': 100, 'distanceToHome': -30, 'distanceToOp': 2}

        # #if all modes are off, encourage our offensive agent to eat more food
        # print "*******Normal Mode********"
        # return {'dead':-9999,'successorScore': 150, 'distanceToFood': -5, 'reverse': -3,'distanceToEntry': -5}

    def evaluate(self, gameState, action):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def dToNearestOp(self, gameState, isPacman=False):
        op_indeics = self.getOpponents(gameState)
        myPos = gameState.getAgentPosition(self.index)
        if op_indeics is None:
            return None
        else:
            minOpDistance = 9999
            for op in op_indeics:
                if not isPacman:
                    if not gameState.getAgentState(op).isPacman:
                        opPos = gameState.getAgentPosition(op)
                        if opPos is not None:
                            opDistance = self.getMazeDistance(myPos, opPos)
                            if opDistance < minOpDistance:
                                minOpDistance = opDistance
                else:
                    if gameState.getAgentState(op).isPacman:
                        opPos = gameState.getAgentPosition(op)
                        if opPos is not None:
                            opDistance = self.getMazeDistance(myPos, opPos)
                            if opDistance < minOpDistance:
                                minOpDistance = opDistance
            if minOpDistance != 9999:
                return minOpDistance
        return None

class DefensiveMCTAgent(MCTCaptureAgent):
  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.boundaryloc, self.searchActions = self.BFS(gameState)
    self.initFood = self.getFoodYouAreDefending(gameState).asList()
    if self.red:
      self.detectDest = [(self.boundaryloc[0]-1,self.boundaryloc[1])]
    else:
      self.detectDest = [(self.boundaryloc[0]+1,self.boundaryloc[1])]


    self.chaseDest = []

  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """

    actions = gameState.getLegalActions(self.index)
    values = [self.evaluate(gameState, a) for a in actions]
   
    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    currEnemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
    currInvader = [enemy for enemy in currEnemies if enemy.isPacman and enemy.getPosition() != None]
    foodList = self.getFoodYouAreDefending(gameState).asList()
    loc = gameState.getAgentState(self.index).getPosition()

    if len(self.initFood) - len(foodList) > 0:
      eatenFood = list(set(self.initFood).difference(set(foodList)))
      self.initFood = foodList
      self.chaseDest = eatenFood

    if len(currInvader) > 0:
      foodLeft = len(self.getFood(gameState).asList())
    elif len(self.chaseDest)>0:
      self.chaseFood = self.AStar(gameState,self.chaseDest[0]) #  chaseFood is several actions return by AStar
      if len(self.chaseFood)>0:          #  How to chase food 
        return self.chaseFood[0]
      if loc == self.chaseDest[0]:       #  The location is the destination location(eaten food location)
        self.chaseDest = []
    #  if didn't detect any invader and didn't find any food has been eaten, then go to defend food
    elif len(self.detectDest)>0:
      # print "my location", loc
      # print "come to the detect destination", self.detectDest
      self.boundaryloc, self.searchActions = self.BFS(gameState)   #  The boudary location AND several actions to boudary location
      if len(self.searchActions)>0:        #  one of the actions to boudary location
        return self.searchActions[0]
      if loc == self.detectDest[0]: 
        self.detectDest=[]
    else:
      self.boundaryloc, self.searchActions = self.BFS(gameState)
      if self.red:
        self.detectDest = [(self.boundaryloc[0] - 1, self.boundaryloc[1])]
      else:
        self.detectDest = [(self.boundaryloc[0] + 1, self.boundaryloc[1])]
      if len(self.searchActions)>0:         #  one of the actions to boudary location
        return self.searchActions[0]

    return random.choice(bestActions)      #  choose best actions, if has same values, random choose one action as best action

  def BFS(self,gameState):
    from util import Queue
    visited, movements, costs, start_cost = [], {}, {}, 0
    startAgentState = gameState.getAgentState(self.index)
    startLoc = startAgentState.getPosition()

    movements[startLoc] = []
    costs[startLoc] = 0
    visited.append(startLoc)

    priorityQueue = Queue()
    priorityQueue.push(gameState)

    while not priorityQueue.isEmpty():
      currGameState = priorityQueue.pop()
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
          priorityQueue.push(succGameState)


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

    # Computes distance to invaders we can see
    enemies = [successorGameState.getAgentState(i) for i in self.getOpponents(successorGameState)]
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
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -100}


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
        self.childs = childs # childs is a list of node

    def addChild(self, c):
        if self.childs is None:
            self.childs = [c]
        else:
            self.childs += [c]
        c.parent = self

    def printNode(self):
        print "Monte Carlo!!!!!!!!"
        print self.gameState
        print "reward: ", self.reward, " cnt: ", self.cnt

    def printChilds(self):
        for child in self.childs:
            child.printNode()