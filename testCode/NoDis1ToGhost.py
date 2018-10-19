    def MCT(self, gameState, iter_times, simulate_depth):

        startTime = time.time()


        root = Node(gameState, 0.0, 0)
        # root does not need to be simulated
        # so add the child directly
        successors = self.generateSuccessors(root.gameState)
        for suc in successors:
            if self.dToNearestOp(suc) > 1:
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
                    if self.dToNearestOp(suc) > 1:
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