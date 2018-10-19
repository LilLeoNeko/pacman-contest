def getNextCrossPoint(self, gameState, step):
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    legalActions = gameState.getLegalActions(self.index)
    usefulActions = list(set(legalActions) - set([Directions.STOP, rev])

    if len(usefulActions) < 2:
        if len(usefulActions) == 1:
            nextAction = usefulActions[0]
            suc = self.getSuccessor(gameState, nextAction)
            return self.getNextCrossPoint(suc, step-1)
        else:
            return None
    return gameState
