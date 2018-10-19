    ##################################################
    # MCT pass a successor to here check whether it leads to empty path
    # Check by dead end identification and score changing
    # dead end & score doesn't change, it is a empty path

    def checkEmptyPath(self, prevGameState, gameState, depth):
        if depth == 0:
            return False
        # check one step
        score = prevGameState.getAgentState(self.index).numCarrying
        oneStepScore = gameState.getAgentState(self.index).numCarrying

        if score < oneStepScore:
            return False

        # the action has taken by successor gamestate
        curDirct = gameState.getAgentState(self.index).configuration.direction
        # create new successor
        successor = gameState.generateSuccessor(self.index, curDirct)
        # print "CHECK BUG HERE", curDirct
        # print "CHECK BUG HERE", successor.getAgentState(self.index)
        sucDirct = successor.getAgentState(self.index).configuration.direction
        revDirct = Directions.REVERSE[sucDirct]
    
        actions = successor.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        if revDirct in actions:
            actions.remove(revDirct)

        if len(actions) == 0:
            print "oh empty dead end"
            return True
        for i in range(depth):
            if not self.checkEmptyPath(gameState, successor, depth-1):
                return False
        return True
    ##################################################