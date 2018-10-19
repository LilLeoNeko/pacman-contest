def simulate(self, gameState, preState, level, R_s, Q_s_a):
      curAction = gameState.getAgentState(self.index).configuration.direction
      rev = Directions.REVERSE[curAction]
      legalActions = gameState.getLegalActions(self.index)
      legalActions.remove(Directions.STOP)
      legalActions.remove(rev)
      #nextAction = random.choice(legalActions)

      suc = self.getSuccessor(gameState, nextAction)
      sucPos = suc.getAgentPosition(self.index)
      levelLeft = level - 1

      max_Q_diff = 0
      max_action = None
      for action in legalActions:
          R_si_ai = self.evaluate(gameState, action)
          if max_Q_diff < R_si_ai:
              max_Q_diff = R_si_ai
              max_act = action

      Q_sp_ap = Q_s_a + max_Q_diff
      preFeatures = self.getFeatures(preState, curAction)
      #curReward += self.evaluate(gameState, nextAction)
      for w_key, w_value in self.weights.items():
          self.weights[w_key] = w_value + self.ALPHA * \
                (R_s + self.GAMMA * max_Q_diff) * preFeatures[w_key]

      if levelLeft > 0:
          return self.simulate(suc, gameState, levelLeft, max_Q_diff, Q_sp_ap)
      return Q_sp_ap
