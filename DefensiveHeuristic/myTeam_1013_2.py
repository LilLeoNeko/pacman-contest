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
        #self.start = gameState.getAgentPosistion(self.index)

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
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

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

        self.noFoodTimer = 0

        # set a survivalPoint that go back home needed
        # reset it to (0,0)
        # update it when is survival mode and survivalPoint is (0,0)
        self.survivalPoint = (0, 0)

        self.start = gameState.getAgentPosition(self.index)
        print "Start: ", self.start
        self.entryPoints = self.homePoints

        self.survivalMode = False
        self.powerMode = False

        self.totalFood = len(self.getFood(gameState).asList())

        self.entryPoints = self.homePoints

        self.isOpRed = not self.red
        x,y = 0,0
        if self.isOpRed:
            x = 1
            y = 1
        else:
            x = gameState.data.layout.width - 2
            y = gameState.data.layout.height - 2

        self.ghostPos = (x,y)

    def chooseAction(self, gameState):
        '''
        You should change this in your own agent.
        '''
        # Get remain food as a list
        foodLeft = len(self.getFood(gameState).asList())
        foodNum = gameState.getAgentState(self.index).numCarrying

        if self.getPreviousObservation() is not None:
            preFoodNum = self.getPreviousObservation().getAgentState(self.index).numCarrying
        else:
            preFoodNum = 0

        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        chaseGhosts = [a for a in enemies if not a.isPacman and a.getPosition() != None and a.scaredTimer is 0]
        if preFoodNum == foodNum and len(chaseGhosts) == 0:
            self.noFoodTimer += 1
        else:
            self.noFoodTimer = 0
        
        # Iteration of Monte Carlo Tree
        # Simulation Depth of Monte Carlo

        CONST_ITERATION_TIMES = 32
        CONST_MONTE_CARLO_DEPTH = 10

        # Check survival mode of pacman
        # if survivalmode needs to be activate
        # check if there is any capsule available within certain distance

        self.survivalCheck(gameState)
        self.powerCheck(gameState)
        if foodNum < 2:
            self.survivalMode = False
        
        """
        if self.survivalMode is True:
            self.takeCapsule(gameState)
        """

        # Timer use to see the performance
        start = time.time()

        # choose actions when survivalmode is off
        if self.survivalMode is False:

            #if self.noFoodTimer < 40:
            MCT = self.MCT(gameState, iter_times=CONST_ITERATION_TIMES,\
                                        simulate_depth=CONST_MONTE_CARLO_DEPTH)

            print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

            childs = MCT.childs
            values = [child.reward/child.cnt for child in childs]
            max_value = max(values)
            nextState = [c for c, v in zip(childs, values) if v == max_value][0]

            return nextState.gameState.getAgentState(self.index).configuration.direction
            '''
            else:
                """
                After a long period didnt eat food,
                go to the food nearest directly
                """
                print "-----Find Food Mode-----"
                action = self.goToFood(gameState)
                print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)
                if action is None:
                    actions = gameState.getLegalActions(self.index)
                    return random.choice(actions)
                else:
                    return action
            '''
        else:
            print "-----Survival Mode-----"
            action = self.goBackHome(gameState)
            print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)
            if action is None:
                actions = gameState.getLegalActions(self.index)
                return random.choice(actions)
            else:
                return action

    # turn on survival mode if ghost(s) chase you
    # turn off if no ghost is chasing you
    # turn off if you are no longer a pacman

    def survivalCheck(self, gameState):
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        chaseGhosts = [a for a in enemies if not a.isPacman and a.getPosition() != None and a.scaredTimer < 5]
        #min([self.getMazeDistance(myState.getPosition(),cg.getPosition()) for cg in chaseGhosts])
        myState = gameState.getAgentState(self.index)
        myPos = myState.getPosition()
        gDist = 9999
        if len(chaseGhosts) > 0:
            gDist = min([self.getMazeDistance(myPos ,cg.getPosition()) for cg in chaseGhosts])
        foodNum = myState.numCarrying
        if myState.isPacman:
            if foodNum > (self.totalFood / 4):
                self.survivalMode = True
            elif len(chaseGhosts) > 0 and gDist <= 5:
                self.survivalMode = True
        else:
            self.survivalMode = False

        # if trun on the survival Mode and survival point is (0,0)
        # update the survival point
        if self.survivalMode and self.survivalPoint == (0, 0):
            print "DO I COME IN?"
            minDis = 9999
            minHP = (0, 0)
            for hp in self.homePoints:
                dis = self.getMazeDistance(myPos, hp)
                if dis < minDis:
                    minDis = dis
                    minHP = hp
            self.survivalPoint = minHP
            print "set the survivalPoint: ", self.survivalPoint

    def powerCheck(self, gameState):
        myState = gameState.getAgentState(self.index)
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]

        minGhostScare = min(enemy.scaredTimer for enemy in enemies)
        if minGhostScare < 15:
            self.powerMode = False
        else:
            self.powerMode = True
            self.survivalMode = False

    def goBackHome(self, gameState):
        actions = gameState.getLegalActions(self.index)
        bestDist = 9999999999999
        bestAction = None
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        ghostPos = [a.getPosition() for a in enemies if not a.isPacman and a.getPosition() != None and a.scaredTimer < 5]
        for action in actions[:]:
            if self.checkDeadEnd(gameState,action,8):
                actions.remove(action)
            print actions
        for action in actions:
            successor = self.getSuccessor(gameState, action)
            pos2 = successor.getAgentPosition(self.index)

            # dist = self.getMazeDistance(self.start,pos2)
            dist = self.getMazeDistance(self.survivalPoint, pos2)

            if len(ghostPos) > 0:
                if pos2 in ghostPos or pos2 == self.start:
                    dist = dist + 99999999
                elif min([self.getMazeDistance(pos2,gp) for gp in ghostPos]) < 2 :
                    dist = dist + 99999999
            if dist < bestDist:
                bestAction = action
                bestDist = dist

            # if already arrived, reset the survivalPoint
            if dist == 0:
                print "reset survivalPoint !", self.survivalPoint
                self.survivalPoint = (0, 0)
                break

        return bestAction

    # Check a dead end while going back home
    # True is a dead end, False is not.

    def checkDeadEnd(self, gameState, action, depth):
        if depth == 0:
            return False
        successor = gameState.generateSuccessor(self.index, action)
        actions = successor.getLegalActions(self.index)
        actions.remove(Directions.STOP)

        curDirct = successor.getAgentState(self.index).configuration.direction
        revDirct = Directions.REVERSE[curDirct]
        if revDirct in actions:
            actions.remove(revDirct)

        if len(actions) == 0:
            return True
        for action in actions:
            if not self.checkDeadEnd(successor, action, depth-1):
                return False
        return True

    ##################################################
    # MCT pass a successor to here check whether it leads to empty path
    # Check by dead end identification and score changing
    # dead end & score doesn't change, it is a empty path

    def checkEmptyPath(self, gameState, action, depth):
        if depth == 0:
            return False
        # check whether score changes
        successor = gameState.generateSuccessor(self.index, action)
        score = gameState.getAgentState(self.index).numCarrying
        newScore = successor.getAgentState(self.index).numCarrying

        if score < newScore:
            return False

        # the action has taken by successor gamestate
        actions = successor.getLegalActions(self.index)
        actions.remove(Directions.STOP)

        curDirct = successor.getAgentState(self.index).configuration.direction
        revDirct = Directions.REVERSE[curDirct]

        if revDirct in actions:
            actions.remove(revDirct)

        if len(actions) == 0:
            return True
        for action in actions:
            if not self.checkEmptyPath(successor, action, depth-1):
                return False
        return True
    ##################################################

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
            curNode = root
            if time.time() - startTime > 0.95:
                break
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
                    #child.printNode()
                    curNode.addChild(child)
                curNode = curNode.childs[0] # random pick a child to simulate

            # part3 simulation
            reward = self.simulate(curNode.gameState, simulate_depth, 0.0)

            # part4 backpropagate
            while curNode.parent is not None:
                curNode.reward += reward
                curNode.cnt += 1
                # print "what happen!!!?: ", curNode.reward
                curNode = curNode.parent
            # root node also need to be updated
            root.reward += reward
            root.cnt += 1

        return root

    def generateSuccessors(self, gameState):
        '''
        generate all successors with legal actions except STOP on the
        current state
        '''
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        ###print "actions: ", actions
        for action in actions:
            if self.checkEmptyPath(gameState, action, 8):
                actions.remove(action)

        return [self.getSuccessor(gameState, action) for action in actions]

    # Input a successorState selected by UCT of last gameState
    # return the reward
    def simulate(self, gameState, level, reward):
        # Get the initial action and next action
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
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
        curReward += self.evaluate(gameState, nextAction,self.survivalMode)

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

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        foodList = self.getFood(successor).asList()
        capsuleList = self.getCapsules(successor)    
        features['successorScore'] = -len(foodList)#self.getScore(successor)
        features['eatenCap'] = -len(capsuleList)

        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        # Compute distance to the nearest food
        myPos = myState.getPosition()
        if len(foodList) > 0: # This should always be True,  but better safe than sorry
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        # Compute distance to the observable enemies
        features['distanceToOp'] = 0
        op_indeics = self.getOpponents(gameState)
        for op in op_indeics:
            if not successor.getAgentState(op).isPacman:
                opPos = successor.getAgentPosition(op)
                if opPos is not None:
                    if self.getMazeDistance(opPos, myPos) == 1:
                        features['distanceToOp'] = -100
                    else:
                        features['distanceToOp'] = self.getMazeDistance(myPos, opPos)

        # BEST_ENTRY

        if not gameState.getAgentState(self.index).isPacman:
            bestEntry = (0,0)
            d_ghostToEntry = 0.0
            for ep in self.entryPoints:
                d = self.getMazeDistance(self.ghostPos,ep)
                if d > d_ghostToEntry:
                    d_ghostToEntry = d
                    bestEntry = ep
            features['distanceToEntry'] = self.getMazeDistance(myPos,bestEntry)
        # BEST_ENTRY END
                
        # Compute distance to the closest home point
        if self.survivalMode is True:
            minHomeDist = min([self.getMazeDistance(myPos,hp) for hp in self.homePoints])
            features['distanceToHome'] = minHomeDist

        # when a pacman arrived home safely
        features['arrivedHome'] = self.getScore(successor)
        
        features['dead'] = 0
        if myPos is self.initialPosition:
            features['dead'] = 1

        features['deadEnd'] = 0
        possibleActions = successor.getLegalActions(self.index)
        if self.survivalMode is True and len(possibleActions) <= 1:
            features['deadEnd'] = 1

        return features

    def getWeights(self, gameState, action):
        if self.powerMode:
            #print "-----Power Mode-----"
            return {'arrivedHome':10, 'successorScore': 150, 'distanceToFood': -5}

        # if survival mode is off, encourage our offensive agent to eat more food
        #print "-----Normal Mode-----"
        return {'arrivedHome': 100, 'successorScore': 150, 'distanceToFood': -5, 'reverse': -3, 'distanceToEntry': -10,'dead': -999, 'deadEnd': -200, 'eatenCap': 200}

    def evaluate(self, gameState, action,survivalMode):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

class DefensiveMCTAgent(MCTCaptureAgent):

  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.initFood = self.getFoodYouAreDefending(gameState).asList()
    self.chaseDest = []
    height = (gameState.data.layout.height - 2) / 2
    i = 0

    # Calculate the central point of the map, and set it as the defender initial position
    if self.red:
      for central in range((gameState.data.layout.width - 2) / 2,0,-1):
        if not gameState.hasWall(central, height):
          self.detectDest = [(central,height)]
          i = i+1
          if i == 2:
            break

    else:
      for central in range((gameState.data.layout.width - 2) / 2 + 1 ,gameState.data.layout.width,1):
        if not gameState.hasWall(central, height):
          self.detectDest = [(central,height)]
          i = i+1
          if i == 2:
            break


  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).

    This function considers the following cases: 
        1.  No invader, no lost food (initial stage).
            Go to the initial defender place.
        2.  Detect there is an invader.
            Go to the invader place to eat it.
        3.  No invader detected, detect lost food. 
            Go to the lost food position(updated regularly).
        4.  Find the invader on the way to the lost food position.
            Give the priority to chase invader.
        5.  Go to enemy territory to eat food during first 30 steps 
        of "scared" situation, otherwise stay in the initial defender 
        position to defend invaders. 

    """

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


    if len(self.initFood) - len(foodList) > 0:
      eatenFood = list(set(self.initFood).difference(set(foodList)))
      self.initFood = foodList
      self.chaseDest = eatenFood
    agentState = gameState.getAgentState(self.index)
    if agentState.scaredTimer > 10: 
      attackValues = [self.evaluateAttack(gameState, a) for a in actions]
      maxAttackValue = max(attackValues)
      bestAttackActions = [a for a, v in zip(actions, attackValues) if v == maxAttackValue]
      return random.choice(bestAttackActions)
    elif agentState.scaredTimer <= 10:
      if len(currInvader) > 0:
        foodLeft = len(self.getFood(gameState).asList())
        self.chaseDest = []
      elif len(self.chaseDest)>0:
        self.chaseFood = self.AStar(gameState,self.chaseDest[0]) 
        if len(self.chaseFood)>0:        
           return self.chaseFood[0]
        if loc == self.chaseDest[0]:      
          self.chaseDest = []
   
      elif len(self.detectDest)>0:
        self.searchActions = self.AStar(gameState, self.detectDest[0])
        if len(self.searchActions)>0:       
          return self.searchActions[0]
    return random.choice(bestActions)      
  

  def AStar(self, gameState,destination):
    """
    Use to calculate the distance to lost food.
    """
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
        return movements[currLoc]
       
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
    """
    Use Manhattan distance to calculate the heuristic distance.
    """
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
    from util import manhattanDistance
    features = util.Counter()
    successorGameState = self.getSuccessor(gameState, action) 
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
      if successorAgentState.scaredTimer > 0 and minDist == 0:
        features['scared'] = 1
        features['stop'] = 0

    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000,
            'onDefense': 1000,
            'invaderDistance': -10,
            'stop': -100,
            'scared': -1000}

  def evaluateAttack(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getAttackFeatures(gameState, action)
    weights = self.getAttackWeights(gameState, action)
    return features * weights

  def getAttackFeatures(self, gameState, action):
    features = util.Counter()
    if action == Directions.STOP: features['stop'] = 1

    succGameState = self.getSuccessor(gameState, action) 
    actions = succGameState.getLegalActions(self.index)
    succAgentState = succGameState.getAgentState(self.index) 
    succPos = succAgentState.getPosition()
    enemies = [succGameState.getAgentState(i) for i in self.getOpponents(succGameState)]
    ghosts = [enemy for enemy in enemies if not enemy.isPacman and enemy.getPosition() != None]
    invaders = [enemy for enemy in enemies if enemy.isPacman and enemy.getPosition() != None]
    capsules = self.getCapsules(succGameState)

    # run away from ghost 
    features['distanceToGhost'] = 99999
    if len(ghosts) > 0:
      minDistToGhost = min([self.getMazeDistance(succPos, ghost.getPosition()) for ghost in ghosts])
      nearestGhost = [ghost for ghost in ghosts if
                      self.getMazeDistance(succPos, ghost.getPosition()) == minDistToGhost]
      if nearestGhost[0].scaredTimer > 0:
        features['distanceToGhost'] = 99999
      elif succAgentState.isPacman:
        features['distanceToGhost'] = minDistToGhost

    # eat capsules
    features['remainCap'] = len(capsules)
    if len(capsules) > 0:
      features['distanceToCap'] = min([self.getMazeDistance(succPos, cap) for cap in capsules])

    # eat food 
    foodList = self.getFood(succGameState).asList()
    features['remainFood'] = len(foodList)  

    if len(foodList) > 0:  # This should always be True,  but better safe than sorry
      features['distanceToFood'] = min([self.getMazeDistance(succPos, food) for food in foodList])

    return features

  def getAttackWeights(self, gameState, action):
    return {
      'remainFood': -100,
      'distanceToFood': -1,
      'remainCap': -100,
      'distanceToCap': -1,
      'distanceToGhost': 1000,
      'stop': -3000,
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