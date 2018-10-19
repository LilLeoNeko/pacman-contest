from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint

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
            self.childs += c
        c.parent = self

def Monte_Carlo_Tree(self, gameState, iter_times, simulate_depth):
    root = Node(gameState, 0.0, 0)

    for i in range(iter_times):
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
                curNode.addChild(child)
            curNode = curNode.childs[0] # random pick a child to simulate

        # part3 simulation
        reward = self.simulate(curNode.gameState, simulate_depth, 0.0)

        # part4 backpropagate
        while curNode.parent is not None:
            curNode.reward += reward
            curNode.cnt += 1
            curNode = curNode.parent
        # root node also need to be updated
        root.reward += reward
        root.cnt += 1

    return root







