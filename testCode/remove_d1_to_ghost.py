def generateSuccessors(self, gameState):
    '''
    generate all successors with legal actions except STOP on the
    current state
    '''
    actions = gameState.getLegalActions(self.index)
    actions.remove(Directions.STOP)

    sucList = [self.getSuccessor(gameState, action) for action in actions]
    for suc in sucList:
        if not self.powerMode and self.dToNearestOp(suc) < 2:
            sucList.remove(suc)
    ###print "actions: ", actions
    return sucList


def simulate(self, gameState, level, reward):
    # Get the initial action and next action
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    legalActions = gameState.getLegalActions(self.index)
    legalActions.remove(Directions.STOP)
    nextAction = random.choice(legalActions)
    suc = self.getSuccessor(gameState, nextAction)

    while not self.powerMode and self.dToNearestOp(suc) < 2:
        legalActions.remove()
        if len(legalActions) < 1:
            return 0
        nextAction = random.choice(legalActions)
        suc = self.getSuccessor(gameState, nextAction)

    # Remove suc state 1D to Ghost

    sucPos = suc.getAgentPosition(self.index)
