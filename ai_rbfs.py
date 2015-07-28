__author__ = 'vivek'

import math, random, sys, time, bisect, string, functools, collections

states_hash = {}

def print_state(state):
    for i in range(3):
        for j in range(3):
            print(state[i][j], end=" "),
        print()


def track_empty_block(state):
    row_index = -1
    column_index = -1

    for i in range(3):
        try:
            row_index = i
            column_index = state[i].index(0)
            break
        except ValueError:
            # print("not found")
            dummy = 0

    if row_index == -1 or column_index == -1:
        print("Error: Couldn't identify the <empty> block")
        exit(0)

    return row_index, column_index

def track_block(state, elem):
    row_index = -1
    column_index = -1

    for i in range(3):
        try:
            row_index = i
            column_index = state[i].index(elem)
            break
        except ValueError:
            # print("not found")
            dummy = 0

    if row_index == -1 or column_index == -1:
        print("Error: Couldn't identify the <empty> block")
        exit(0)

    return row_index, column_index


class Problem(object):
    """The abstract class for a formal problem.  You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        return ['up', 'down', 'left', 'right']

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        if action in self.actions():
            empty_block_row, empty_block_column = track_empty_block(state)
            if action == 'up':
                if empty_block_row in range(1, 3):
                    temp = state[empty_block_row-1][empty_block_column]
                    state[empty_block_row][empty_block_column] = temp
                    state[empty_block_row-1][empty_block_column] = 0
                else:
                    state = 0
            elif action == 'down':
                if empty_block_row in range(0, 2):
                    temp = state[empty_block_row+1][empty_block_column]
                    state[empty_block_row][empty_block_column] = temp
                    state[empty_block_row+1][empty_block_column] = 0
                else:
                    state = 0
            elif action == 'left':
                if empty_block_column in range(1, 3):
                    temp = state[empty_block_row][empty_block_column-1]
                    state[empty_block_row][empty_block_column] = temp
                    state[empty_block_row][empty_block_column-1] = 0
                else:
                    state = 0
            elif action == 'right':
                if empty_block_column in range(0, 2):
                    temp = state[empty_block_row][empty_block_column+1]
                    state[empty_block_row][empty_block_column] = temp
                    state[empty_block_row][empty_block_column+1] = 0
                else:
                    state = 0

        return state

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Override this
        method if checking against a single self.goal is not enough."""
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        abstract

    def h(self, state):

        heuristic_distance = 0
        for i in range(3):
            for j in range(3):
                block_row, block_column = track_block(state, self.goal[i][j])
                heuristic_distance += abs(i - block_row) + abs(j - block_column)
        return heuristic_distance
#______________________________________________________________________________


class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    f=0

    def __init__(self, state, parent=None, action=None, path_cost=0):
        #"Create a search tree Node, derived from a parent by an action."
        # update(self, state, parent, action, path_cost)
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

        if parent:
            self.depth = parent.depth + 1
        else:
            self.depth = 0

        # states_hash[self] = self.__key()

    def __key(self):
        state_str = ''
        for row in self.state:
            state_str = state_str + " ".join(map(str, row)) + " "

        return state_str


    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def expand(self, problem):
        # "List the nodes reachable in one step from this node."
        list_nodes = []
        for action in problem.actions():
            new_node = self.child_node(problem, action)
            if new_node != 0:
                list_nodes.append(new_node)

        return list_nodes

    def child_node(self, problem, action):
        value_state = [[0 for i in range(3)] for j in range(3)]
        for i in range(3):
            for j in range(3):
                value_state[i][j] = self.state[i][j]
        next = problem.result(value_state, action)

        if next != 0:
            state_str = ''
            for row in next:
                state_str = state_str + " ".join(map(str, row)) + " "

            # if state_str in states_hash.values():
            #     print("duplicate")

            new_node = Node(next, self, action, self.path_cost+1)
        else:
            new_node = 0

        return new_node

    def solution(self):
        "Return the sequence of actions to go from the root to this node."
        return [node.action for node in self.path()[1:]]

    def path(self):
        "Return a list of nodes forming the path from the root to this node."
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.__key() == other.__key()

    def __hash__(self):
        return hash(self.__key())

#______________________________________________________________________________
def custom_sort(node):
    return node.f

def recursive_best_first_search(problem, h=None):

    def RBFS(problem, node, flimit):
        print("depth:", node.depth, "f:", node.f)
        print_state(node.state)
        if problem.goal_test(node.state):
            return node, 0   # (The second value is immaterial)
        successors = node.expand(problem)
        if len(successors) == 0:
            if node.f == 5000:
                return "Fail", 5000
            else:
                return None, 5000

        print("successors...")
        for s in successors:
            s.f = max(s.path_cost + problem.h(s.state), node.f)
            print("h:", problem.h(s.state), "g:", s.path_cost, "f:", s.f)
            print_state(s.state)

        while True:
            sorted_successors = sorted(successors, key=custom_sort)

            best = sorted_successors[0]
            if best.f > flimit:
                print("current best:%d, alternate best: %d"%(best.f, flimit))
                return None, best.f
            if len(sorted_successors) > 1:
                alternative = sorted_successors[1].f
            else:
                alternative = 5000
            print("chosen state:")
            print_state(best.state)
            print("\n")
            result, best.f = RBFS(problem, best, min(flimit, alternative))
            print("Back tracked ...")
            if result is not None:
                return result, best.f

    node = Node(problem.initial)

    state_str = ''
    for row in node.state:
        state_str = state_str + " ".join(map(str, row)) + " "

    node.f = problem.h(node.state)
    result, bestf = RBFS(problem, node, 5000)

    return result

print("Start ...")

fh = open("input.txt", "rU")

initial_state = [[0 for i in range(3)] for j in range(3)]
line_no = 0
for line in fh:
    lst_line = line.split()
    for i in range(3):
        initial_state[line_no][i] = int(lst_line[i])
    line_no += 1
    if line_no >= 3:
        break

for i in range(3):
    for j in range(3):
        if initial_state[i][j] < 0 or initial_state[i][j] > 8:
            print("Error in input")
            exit(0)

print("Initial state:")
print_state(initial_state)

goal_state = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
print("Goal state:")
print_state(goal_state)

puzzle = Problem(initial_state, goal_state)
result = recursive_best_first_search(puzzle)

if isinstance(result, Node):
    print("\nSolution for the 8 puzzle:")
    action_list = result.solution()
    print_state(initial_state)
    print("")
    i = 1
    for step in action_list:
        print("%d) %s"%(i, step))
        print_state(puzzle.result(initial_state, step))
        print("")
        i += 1
else:
    print(result)

print("End.")