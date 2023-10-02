# search.py
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, listOfActions):
        """
         listOfActions: A list of listOfActions to take

        This method returns the total cost of a particular sequence of listOfActions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of listOfActions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    
    #the fringe for dfs is a stack
    fringe = util.Stack()

    #list of nodes we have seen, starts empty
    seen = []

    #list of actions, starts empty
    listOfActions = []

    #start with setting up the first node
    startState = problem.getStartState()
    startNode = (startState, listOfActions)
    fringe.push(startNode)
    
    #if the fringe has elements in it, pop it, then see if we have already seen the state
    #if we havent seen the state, add it to the seen list
    #check if it is the goal state, if it is then return the list of actions required to get there
    #if it isnt, then get the successors, find the new action, create a new node with it, and then add it to the fringe
    #once this is done, return the list of actions
    while not fringe.isEmpty():
        currentState, listOfActions = fringe.pop()
        
        if currentState not in seen:
            seen.append(currentState)

            if problem.isGoalState(currentState):
                return listOfActions
            else:
                successors = problem.getSuccessors(currentState)
                
                for successorState, successorAction, successorCost in successors:
                    newAction = listOfActions + [successorAction]
                    newNode = (successorState, newAction)
                    fringe.push(newNode)

    return listOfActions  


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    #fringe is a queue in bfs
    fringe = util.Queue()
    
    #list of nodes we have seen, starts empty
    seen = []

    #list of actions, starts empty
    listOfActions = []

    #start with setting up the first node
    startState = problem.getStartState()
    startNode = (startState, listOfActions, 0)
    fringe.push(startNode)
    
    #while there are nodes in the fringe, pop the fringe. if we have not seen the state, add it to seen
    #if the current state is the goal state, return the actions needed to get there
    #if it isnt, then get the successors, find the new action, create a new node with it, and then add it to the fringe
    #once this is done, return the list of actions
    while not fringe.isEmpty():
        currentState, listOfActions, currentCost = fringe.pop()
        
        if currentState not in seen:
            seen.append(currentState)

            if problem.isGoalState(currentState):
                return listOfActions
            else:
                successors = problem.getSuccessors(currentState)
                
                for successorState, successorAction, successorCost in successors:
                    newAction = listOfActions + [successorAction]
                    newCost = currentCost + successorCost
                    newNode = (successorState, newAction, newCost)
                    fringe.push(newNode)

    return listOfActions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    #fringe for ucs is a priority queue
    fringe = util.PriorityQueue()

    #list of nodes we have seen, we get the data from the [state], starts empty
    seen = {}

    #list of actions we have done, starts empty
    listOfActions = []

    #set up the start state, starts with 0 cost
    startState = problem.getStartState()
    startNode = (startState, listOfActions, 0) 
    fringe.push(startNode, 0)
    
    #while there are nodes in the fringe, pop the fringe. if we have not seen the state or the current cost is less 
    # than the cost of this state that we have already seen, make seen at the current state equal to the current cost
    #if the current state is the goal state, return the actions needed to get there
    #if it isnt, then get the successors, find the new action, create a new node with it, and then add it to the fringe
    #once this is done, return the list of actions
    while not fringe.isEmpty():
        currentState, listOfActions, currentCost = fringe.pop()
       
        if (currentState not in seen) or (currentCost < seen[currentState]):
            seen[currentState] = currentCost

            if problem.isGoalState(currentState):
                return listOfActions
            else:
                successors = problem.getSuccessors(currentState)
                
                for successorState, successorAction, successorCost in successors:
                    newAction = listOfActions + [successorAction]
                    newCost = currentCost + successorCost
                    newNode = (successorState, newAction, newCost)

                    fringe.update(newNode, newCost)

    return listOfActions


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    #fringe is a priority queue
    fringe = util.PriorityQueue()

    #list of nodes we have seen, starts empty
    seen = [] 

    #list of actions we have done, starts empty
    listOfActions = []

    #set up start state
    startState = problem.getStartState()
    startNode = (startState, listOfActions, 0) 
    fringe.push(startNode, 0)

    #while there are nodes in the fringe, pop the fringe. then, add the current state with its cost to the list
    #if the current state is the goal state, return the actions needed to get there
    #if it isnt, then get the successors, find the new action, cost, create a new node with it 
    # for each element that we have seen, if the successor's cost is >= the current cost and both
    #the successor and the current node do not have the same state, then 
    #push this new node onto the fringe and then add the successors state with this new cost to the seen list
    #once this is done, return the list of actions
    while not fringe.isEmpty():
        currentState, listOfActions, currentCost = fringe.pop()
        seen.append((currentState, currentCost))

        if problem.isGoalState(currentState):
            return listOfActions

        else:
            successors = problem.getSuccessors(currentState)
            
            for successorState, successorAction, successorCost in successors:
                newAction = listOfActions + [successorAction]
                newCost = problem.getCostOfActions(newAction)
                newNode = (successorState, newAction, newCost)

                haveSeen = False

                for eachSeen in seen:
                    seenState, seenCost = eachSeen

                    if ((successorState == seenState) and (newCost >= seenCost)):
                        haveSeen = True

                if not haveSeen:
                    fringe.push(newNode, newCost + heuristic(successorState, problem))
                    seen.append((successorState, newCost))

    return listOfActions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
