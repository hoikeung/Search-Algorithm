
class Tree(object):
    
    board = []
    parent = None
    deep = None
    movement = None
    cost = None
    UDLR_priority = None
    h_of_n = None
    limit = None

    def __init__(self, board = None, parent = None, deep = None, movement = None, cost = None, UDLR_priority = None, h_of_n = None, limit = None):
        self.board = board
        self.parent = parent
        self.deep = deep
        self.movement = movement
        self.cost = cost
        self.UDLR_priority = UDLR_priority
        self.h_of_n = h_of_n
        self.limit = limit
        
    def __lt__(self, other):
        if (self.limit != None):
            return (self.limit, self.cost, self.h_of_n, self.UDLR_priority) < (self.limit, other.cost, other.h_of_n, other.UDLR_priority)
        else:
            return(self.cost, self.h_of_n, self.UDLR_priority) < (other.cost, other.h_of_n, other.UDLR_priority)

import timeit
import copy
import os
import psutil
import heapq
from queue import PriorityQueue
import sys

#board size
ROW = 3
COLUMN = 3

#movement
Up = 'Up'
Down = 'Down'
Left = 'Left'
Right = 'Right'

#goal
goal_state = [[0,1,2], [3,4,5], [6,7,8]]
goal_dict = {0:[0,0], 1:[0,1], 2:[0,2], 3:[1,0], 4:[1,1], 5:[1,2], 6:[2,0], 7:[2,1], 8:[2,2]}

#output.txt required parameters
path_to_goal = []
cost_of_path = 0 
nodes_expanded = 0
fringe_size = 0
max_fringe_size = 0
search_depth = 0
max_search_depth = 0
running_time = 0
max_ram_usage = 0

search_limit_bound = 6

def bfs(*number_list):
    
    start_time = timeit.default_timer()
        
    init_board = Init_State(number_list)
    init_state = Tree(board = init_board, deep = 0)
    
    frontier = []
    explored = set()
    
    global fringe_size
    global max_fringe_size
    global search_depth
    global max_search_depth
    
    frontier.append(init_state)
    fringe_size = 1
    
    while frontier:
        current_state = frontier[0]
        
        frontier.pop(0)
        fringe_size = fringe_size - 1
        
        explored.add(str(current_state.board))
        
        if (current_state.board == goal_state):
            Find_Path(current_state)
            
            search_depth = current_state.deep
            
            cost_of_path = len(path_to_goal)
            
            max_ram_usage = psutil.Process(os.getpid()).memory_info().rss / 2.**20
            running_time = timeit.default_timer() - start_time
            
            f = open('output.txt', 'w')
            f.write("path_to_goal: " + str(path_to_goal) + "\n")
            f.write("cost_of_path: " + str(cost_of_path) + "\n")
            f.write("nodes_expanded: " + str(nodes_expanded) + "\n")
            f.write("fringe_size: " + str(fringe_size) + "\n")
            f.write("max_fringe_size: " + str(max_fringe_size) + "\n")
            f.write("search_depth: " + str(search_depth) + "\n")
            f.write("max_search_depth: " + str(max_search_depth) + "\n")
            f.write("running_time: " + str(running_time) + "\n")
            f.write("max_ram_usage: " + str(max_ram_usage) + "\n")
            
            return current_state.board

        Explore_Node_BFS(current_state, frontier, explored)
       
    return [0,0,0,0,0,0,0,0,0]

def dfs(*number_list):
    
    start_time = timeit.default_timer()
        
    init_board = Init_State(number_list)
    init_state = Tree(board = init_board, deep = 0)
    
    frontier = []
    explored = set()
    
    global fringe_size
    global max_fringe_size
    global search_depth
    global max_search_depth
    
    frontier.append(init_state)
    fringe_size = 1
    
    while frontier:
        current_state = frontier[-1]
        
        frontier.pop(-1)
        fringe_size = fringe_size - 1
        
        explored.add(str(current_state.board))
        
        if (current_state.board == goal_state):
            Find_Path(current_state)
            
            search_depth = current_state.deep
            
            cost_of_path = len(path_to_goal)
            
            max_ram_usage = psutil.Process(os.getpid()).memory_info().rss / 2.**20
            running_time = timeit.default_timer() - start_time
            
            f = open('output.txt', 'w')
            f.write("path_to_goal: " + str(path_to_goal) + "\n")
            f.write("cost_of_path: " + str(cost_of_path) + "\n")
            f.write("nodes_expanded: " + str(nodes_expanded) + "\n")
            f.write("fringe_size: " + str(fringe_size) + "\n")
            f.write("max_fringe_size: " + str(max_fringe_size) + "\n")
            f.write("search_depth: " + str(search_depth) + "\n")
            f.write("max_search_depth: " + str(max_search_depth) + "\n")
            f.write("running_time: " + str(running_time) + "\n")
            f.write("max_ram_usage: " + str(max_ram_usage) + "\n")
            
            return current_state.board

        Explore_Node_DFS(current_state, frontier, explored)
       
    return [0,0,0,0,0,0,0,0,0]

def ast(*number_list):
    start_time = timeit.default_timer()
        
    init_board = Init_State(number_list)
    init_state = Tree(board = init_board, deep = 0)
    init_state.cost = Find_Cost(init_state.board)
    init_state.UDLR_priority = 1
    
    frontier = []
    explored = set()
    
    global fringe_size
    global max_fringe_size
    global search_depth
    global max_search_depth
    
    heapq.heappush(frontier, init_state)
    fringe_size = 1
    
    while (frontier):
        current_state = heapq.heappop(frontier)
        fringe_size = fringe_size - 1
        
        explored.add(str(current_state.board))
          
        if (current_state.board == goal_state):
            Find_Path(current_state)
            
            search_depth = current_state.deep
            
            cost_of_path = len(path_to_goal)
            
            max_ram_usage = psutil.Process(os.getpid()).memory_info().rss / 2.**20
            running_time = timeit.default_timer() - start_time
            
            f = open('output.txt', 'w')
            f.write("path_to_goal: " + str(path_to_goal) + "\n")
            f.write("cost_of_path: " + str(cost_of_path) + "\n")
            f.write("nodes_expanded: " + str(nodes_expanded) + "\n")
            f.write("fringe_size: " + str(fringe_size) + "\n")
            f.write("max_fringe_size: " + str(max_fringe_size) + "\n")
            f.write("search_depth: " + str(search_depth) + "\n")
            f.write("max_search_depth: " + str(max_search_depth) + "\n")
            f.write("running_time: " + str(running_time) + "\n")
            f.write("max_ram_usage: " + str(max_ram_usage) + "\n")
            
            return current_state.board

        Explore_Node_AST(current_state, frontier, explored)
        
    return [0,0,0,0,0,0,0,0,0]

def ida(*number_list):
    start_time = timeit.default_timer()
        
    init_board = Init_State(number_list)
    init_state = Tree(board = init_board, deep = 0)
    init_state.cost = Find_Cost(init_state.board)
    init_state.UDLR_priority = 1
    
    search_limit = 0
    
    frontier = []
    explored = set()
    
    global fringe_size
    global max_fringe_size
    global search_depth
    global max_search_depth
    
    heapq.heappush(frontier, init_state)
    fringe_size = 1
    
    while (frontier):
        current_state = heapq.heappop(frontier)
        fringe_size = fringe_size - 1
        
        explored.add(str(current_state.board))
          
        if (current_state.board == goal_state):
            Find_Path(current_state)
            
            search_depth = current_state.deep
            
            cost_of_path = len(path_to_goal)
            
            max_ram_usage = psutil.Process(os.getpid()).memory_info().rss / 2.**20
            running_time = timeit.default_timer() - start_time
            
            f = open('output.txt', 'w')
            f.write("path_to_goal: " + str(path_to_goal) + "\n")
            f.write("cost_of_path: " + str(cost_of_path) + "\n")
            f.write("nodes_expanded: " + str(nodes_expanded) + "\n")
            f.write("fringe_size: " + str(fringe_size) + "\n")
            f.write("max_fringe_size: " + str(max_fringe_size) + "\n")
            f.write("search_depth: " + str(search_depth) + "\n")
            f.write("max_search_depth: " + str(max_search_depth) + "\n")
            f.write("running_time: " + str(running_time) + "\n")
            f.write("max_ram_usage: " + str(max_ram_usage) + "\n")
            
            return current_state.board
        
        if (not frontier) and (search_limit > 0):
            search_limit = current_state.cost + 10
        
        Explore_Node_IDA(current_state, frontier, explored, search_limit)
        
    return [0,0,0,0,0,0,0,0,0]

def Init_State(number_list):
    state = []
    count = 0
    
    for i in range(ROW):
        row = []
        
        for j in range(COLUMN):
            row.append(number_list[count])
            count = count + 1
            
        state.append(row)
        
    return state

def Get_Position(my_list, number = None):
    value = None
    
    if number == None:
        value = 0
    else:
        value = number
        
    for i, x in enumerate(my_list):
        if value in x:
            return (i, x.index(value))

def Explore_Node_BFS(c, f, e):
    global nodes_expanded
    global fringe_size
    global max_fringe_size
    global max_search_depth
    
    nodes_expanded = nodes_expanded + 1
    
    i, j = Get_Position(c.board)
    
    #move up
    if (i > 0):
        result = copy.deepcopy(c.board)
        temp_number = result[i-1][j]
        result[i-1][j] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            up = Tree(parent = c, deep = c.deep + 1)
            up.board = result
            up.movement = Up
            f.append(up)
            e.add(str(result))
    
    #move down
    if (i < ROW - 1):
        result = copy.deepcopy(c.board)
        temp_number = result[i+1][j]
        result[i+1][j] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            down = Tree(parent = c, deep = c.deep + 1)
            down.board = result
            down.movement = Down
            f.append(down)
            e.add(str(result))
    
    #move left
    if (j > 0):
        result = copy.deepcopy(c.board)
        temp_number = result[i][j-1]
        result[i][j-1] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            left = Tree(parent = c, deep = c.deep + 1)
            left.board = result
            left.movement = Left
            f.append(left)
            e.add(str(result))
    
    #move right
    if (j < COLUMN - 1):
        result = copy.deepcopy(c.board)
        temp_number = result[i][j+1]
        result[i][j+1] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            right = Tree(parent = c, deep = c.deep + 1)
            right.board = result
            right.movement = Right
            f.append(right)
            e.add(str(result))
    
    fringe_size = len(f)
    
    if (fringe_size > max_fringe_size):
        max_fringe_size = fringe_size
    
    if (f[-1].deep > max_search_depth):
        max_search_depth = f[-1].deep

def Explore_Node_DFS(c, f, e):
    global nodes_expanded
    global fringe_size
    global max_fringe_size
    global max_search_depth
    
    nodes_expanded = nodes_expanded + 1
    
    i, j = Get_Position(c.board)
    
    #move right
    if (j < COLUMN - 1):
        result = copy.deepcopy(c.board)
        temp_number = result[i][j+1]
        result[i][j+1] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            right = Tree(parent = c, deep = c.deep + 1)
            right.board = result
            right.movement = Right
            f.append(right)
            e.add(str(result))
    
    #move left
    if (j > 0):
        result = copy.deepcopy(c.board)
        temp_number = result[i][j-1]
        result[i][j-1] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            left = Tree(parent = c, deep = c.deep + 1)
            left.board = result
            left.movement = Left
            f.append(left)
            e.add(str(result))

    #move down
    if (i < ROW - 1):
        result = copy.deepcopy(c.board)
        temp_number = result[i+1][j]
        result[i+1][j] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            down = Tree(parent = c, deep = c.deep + 1)
            down.board = result
            down.movement = Down
            f.append(down)
            e.add(str(result))

    #move up
    if (i > 0):
        result = copy.deepcopy(c.board)
        temp_number = result[i-1][j]
        result[i-1][j] = 0
        result[i][j] = temp_number
        
        if (not (str(result) in e)):
            up = Tree(parent = c, deep = c.deep + 1)
            up.board = result
            up.movement = Up
            f.append(up)
            e.add(str(result))
  
    fringe_size = len(f)
    
    if (fringe_size > max_fringe_size):
        max_fringe_size = fringe_size
    
    if (f[-1].deep > max_search_depth):
        max_search_depth = f[-1].deep

def Find_Path(c):
    find_path = c
    
    while (find_path.parent != None):
        path_to_goal.append(find_path.movement)
        find_path = find_path.parent
    
    path_to_goal.reverse()

def Find_Cost(board):
    global goal_dict
    
    number = 1
    
    cost = 0
    
    while (number < 9):
        i, j = Get_Position(board, number)
        index_list = [i, j]
        
        if (index_list != goal_dict[number]):
            cost = cost + abs(i - goal_dict[number][0]) + abs(j - goal_dict[number][1])
        
        number = number + 1
    
    return cost

def Explore_Node_AST(c, f, e):
    global nodes_expanded
    global fringe_size
    global max_fringe_size
    global max_search_depth
    
    nodes_expanded = nodes_expanded + 1
    
    i, j = Get_Position(c.board)
    up = None
    down = None
    left = None
    right = None
    
    #move up
    if (i > 0):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i-1][j]
        result[i-1][j] = 0
        result[i][j] = temp_number
        
        up = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 1, movement = Up)
        up.h_of_n = Find_Cost(up.board)
        up.cost = c.deep + up.h_of_n

        for x in f:
            if (x.board == result):
                in_f = True
                
                if (x.cost > up.cost):
                    f.remove(x)
                    heapq.heappush(f, up)
                #else:
                    #in_f = False
                    #print ("else")
        
        if ((not in_f) and (not (str(result) in e))):
            heapq.heappush(f, up)
    
    #move down
    if (i < ROW - 1):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i+1][j]
        result[i+1][j] = 0
        result[i][j] = temp_number
        
        down = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 2, movement = Down)
        down.h_of_n = Find_Cost(down.board)
        down.cost = c.deep + down.h_of_n
        
        for x in f:
            if (x.board == result):
                in_f = True
                
                if (x.cost > down.cost):
                    f.remove(x)
                    heapq.heappush(f, down)
                #else:
                    #in_f = False
        
        if ((not in_f) and (not (str(result) in e))):
            heapq.heappush(f, down)
    
    #move left
    if (j > 0):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i][j-1]
        result[i][j-1] = 0
        result[i][j] = temp_number
        
        left = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 3, movement = Left)
        left.h_of_n = Find_Cost(left.board)
        left.cost = c.deep + left.h_of_n
        
        for x in f:
            if (x.board == result):
                in_f = True
                
                if (x.cost > left.cost):
                    f.remove(x)
                    heapq.heappush(f, left)
                #else:
                    #in_f = False
        
        if ((not in_f) and (not (str(result) in e))):
            heapq.heappush(f, left)
        
    #move right
    if (j < COLUMN - 1):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i][j+1]
        result[i][j+1] = 0
        result[i][j] = temp_number
        
        right = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 4, movement = Right)
        right.h_of_n = Find_Cost(right.board)
        right.cost = c.deep + right.h_of_n
        
        for x in f:
            if (x.board == result):
                in_f = True
                
                if (x.cost > right.cost):
                    f.remove(x)
                    heapq.heappush(f, right)
                    
                #else:
                    #in_f = False
        
        if ((not in_f) and (not (str(result) in e))):
            heapq.heappush(f, right)
            
    fringe_size = len(f)
    
    if (fringe_size > max_fringe_size):
        max_fringe_size = fringe_size
    
    if up != None:
        if (up.deep > max_search_depth):
            max_search_depth = up.deep
    elif down != None:
        if (down.deep > max_search_depth):
            max_search_depth = down.deep
    elif left != None:
        if (left.deep > max_search_depth):
            max_search_depth = left.deep
    elif right != None:
        if (right.deep > max_search_depth):
            max_search_depth = right.deep
    
def Explore_Node_IDA(c, f, e, s):
    global nodes_expanded
    global fringe_size
    global max_fringe_size
    global max_search_depth
    
    global search_limit_bound
    
    nodes_expanded = nodes_expanded + 1
    
    i, j = Get_Position(c.board)
    up = None
    down = None
    left = None
    right = None
    
    #move up
    if (i > 0):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i-1][j]
        result[i-1][j] = 0
        result[i][j] = temp_number
        
        up = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 1, movement = Up)
        up.h_of_n = Find_Cost(up.board)
        up.cost = c.deep + up.h_of_n
        up.limit = s - up.cost
        
        if (up.limit <= search_limit_bound):
            for x in f:
                if (x.board == result):
                    in_f = True
                    
                    if (x.cost > up.cost):
                        f.remove(x)
                        heapq.heappush(f, up)
                    #else:
                        #in_f = False
            
            if ((not in_f) and (not (str(result) in e))):
                heapq.heappush(f, up)
        
        else:
            up = None
    
    #move down
    if (i < ROW - 1):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i+1][j]
        result[i+1][j] = 0
        result[i][j] = temp_number
        
        down = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 2, movement = Down)
        down.h_of_n = Find_Cost(down.board)
        down.cost = c.deep + down.h_of_n
        down.limit = s - down.cost
        
        if (down.limit < search_limit_bound):
            for x in f:
                if (x.board == result):
                    in_f = True
                    
                    if (x.cost > down.cost):
                        f.remove(x)
                        heapq.heappush(f, down)
                    #else:
                        #in_f = False
            
            if ((not in_f) and (not (str(result) in e))):
                heapq.heappush(f, down)
         
        else:
            down = None
            
    #move left
    if (j > 0):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i][j-1]
        result[i][j-1] = 0
        result[i][j] = temp_number
        
        left = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 3, movement = Left)
        left.h_of_n = Find_Cost(left.board)
        left.cost = c.deep + left.h_of_n
        left.limit = s - left.cost
        
        if (left.limit < search_limit_bound):
            for x in f:
                if (x.board == result):
                    in_f = True
                    
                    if (x.cost > left.cost):
                        f.remove(x)
                        heapq.heappush(f, left)
                    #else:
                        #in_f = False
            
            if ((not in_f) and (not (str(result) in e))):
                heapq.heappush(f, left)
        
        else:
            left = None
            
    #move right
    if (j < COLUMN - 1):
        in_f = False

        result = copy.deepcopy(c.board)
        temp_number = result[i][j+1]
        result[i][j+1] = 0
        result[i][j] = temp_number
        
        right = Tree(parent = c, deep = c.deep + 1, board = result, UDLR_priority = 4, movement = Right)
        right.h_of_n = Find_Cost(right.board)
        right.cost = c.deep + right.h_of_n
        right.limit = s - right.cost
        
        if (right.limit < 6):
            if (right.limit < 6):
                for x in f:
                    if (x.board == result):
                        in_f = True
                        
                        if (x.cost > right.cost):
                            f.remove(x)
                            heapq.heappush(f, right)
                            
                        #else:
                            #in_f = False
                
            if ((not in_f) and (not (str(result) in e))):
                heapq.heappush(f, right)
        
        else:
            right = None
                
    fringe_size = len(f)
    
    if (fringe_size > max_fringe_size):
        max_fringe_size = fringe_size
    
    if up != None:
        if (up.deep > max_search_depth):
            max_search_depth = up.deep
    elif down != None:
        if (down.deep > max_search_depth):
            max_search_depth = down.deep
    elif left != None:
        if (left.deep > max_search_depth):
            max_search_depth = left.deep
    elif right != None:
        if (right.deep > max_search_depth):
            max_search_depth = right.deep



#test = bfs(7,2,4,5,0,6,8,3,1)
#print ("test = " + str(test))

f_call = sys.argv[1]
str_list = sys.argv[2]

int_list = tuple(int(s) for s in str_list.split(','))

if (f_call == "bfs"):
    bfs(int_list[0],int_list[1],int_list[2],int_list[3],int_list[4],int_list[5],int_list[6],int_list[7],int_list[8])
elif (f_call == "dfs"):
    dfs(int_list[0],int_list[1],int_list[2],int_list[3],int_list[4],int_list[5],int_list[6],int_list[7],int_list[8])
elif (f_call == "ast"):
    ast(int_list[0],int_list[1],int_list[2],int_list[3],int_list[4],int_list[5],int_list[6],int_list[7],int_list[8])
elif (f_call == "ida"):
    ida(int_list[0],int_list[1],int_list[2],int_list[3],int_list[4],int_list[5],int_list[6],int_list[7],int_list[8])
    
    