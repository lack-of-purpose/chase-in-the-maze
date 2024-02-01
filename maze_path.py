from queue import PriorityQueue

class Node:
    """
    A class used to represent a cell (node) in maze (graph)

    ...

    Attributes
    ----------
    node : int
        a number of the cell
    cost : int
        the cost of coming to the node
    move : Edge
        possible move from the cell

    """
    def __init__(self, node, cost=None, move=None):
        self.node = node
        if cost != None:
            self.cost = cost
        if move != None:
            self.move = move
    
    def __eq__(self, other):
        if not isinstance(other, Node):
            return NotImplemented
        return self.cost == other.cost
    
    def __lt__(self, other):
        if not isinstance(other, Node):
            return NotImplemented
        return self.cost < other.cost
    
    def __gt__(self, other):
        if not isinstance(other, Node):
            return NotImplemented
        return self.cost > other.cost
        
class Edge:
    """
    A class used to represent a move (edge) in a maze (graph) between two cells

    ...

    Attributes
    ----------
    dest : int
        the number of the target cell
    weight : int
        the weight of the edge
    dir : int
        the direction of the edge (0: up, 1: right, 2: down, 3: left)

    """
    def __init__(self, dest, weight, dir):
        self.dest = dest
        self.weight = weight
        self.dir = dir

class Maze:
    """
    A class used to represent a state of a maze

    ...

    Attributes
    ----------
    width : int
        the width of the maze
    height : int
        the height of the maze
    monster : int
        the position of the monster (m_i*width+m_j)
    (m_i, m_j) : (int, int)
        monster coordinates
    init_state : int
        the position of the P (p_i*width+p_j)
    (p_i, p_j) : (int, int)
        P coordinates
    goal : int
        the position of the exit (g_i*width+g_j)
    (g_i, g_j) : (int, int)
        exit coordinates
    maze : {node : [moves]}
        maze representation as a dictionary of cells as keys and lists of possible moves from the cell as values

    Methods
    -------
    monster_move(p_i, p_j, m_i, m_j)
        Returns the next coordinates of the monster after moving based on P coordinates.
    find_position(maze_matrix, element)
        Returns the position of the cell containing the given element (E, M, etc).
    get_neighbors(i, j, maze_matrix)
        Returns all neighbors for the given cell coordinates where move is possible, and the direction of that move.
    fill_the_maze(maze_matrix)
        Creates and returns the maze.
    """
    def __init__(self, maze_matrix, monster=False):
        self.width = len(maze_matrix[0])
        self.height = len(maze_matrix)
        self.monster, (self.m_i, self.m_j) = self.find_position(maze_matrix, 'M')
        self.init_state, (self.init_i, self.init_j) = self.find_position(maze_matrix, 'P')
        self.goal, (self.g_i, self.g_j) = self.find_position(maze_matrix, 'E')
        if monster:
            maze_matrix[self.m_i][self.m_j] = '.'
            (self.m_i, self.m_j) = self.monster_move(self.init_i, self.init_j, self.m_i, self.m_j, maze_matrix)
            self.monster = self.m_i*self.width + self.m_j
            maze_matrix[self.m_i][self.m_j] = 'M'
        self.maze = self.fill_the_maze(maze_matrix)
        
    def monster_move(self, p_i, p_j, m_i, m_j, maze_matrix):
        """Returns the next coordinates of the monster after moving based on P coordinates.

        Parameters
        ----------
        p_i, p_j : int, int
            Coordinates of P
        m_i, m_j : int, int
            Coordinates of monster
        maze_matrix : []
            Matrix representation of the maze

        Returns
        ------
        tuple
            a tuple of new monster coordinates
        """
        neighbors = self.get_neighbors(m_i, m_j, maze_matrix)

        if p_j < m_j and (m_i, m_j-1, 3) in neighbors:
            return (m_i, m_j-1)
        elif p_j > m_j and (m_i, m_j+1, 1) in neighbors:
            return (m_i, m_j + 1)
        elif p_i < m_i and (m_i-1, m_j, 0) in neighbors:
            return (m_i-1, m_j)
        elif p_i > m_i and (m_i+1, m_j, 2) in neighbors:
            return (m_i+1, m_j)
        return (m_i, m_j)
      
    def find_position(self, maze_matrix, element):
        """Returns the number of the cell containing the given element (E, M, etc).

        Parameters
        ----------
        maze_matrix : []
            Matrix representation of the maze
        element : char
            E, M or P

        Returns
        ------
        int, tuple
            an int number of the cell and tuple with its coordinates
        """
        for i in range(self.height):
            for j in range(self.width):
                if maze_matrix[i][j] == element:
                    coords = (i, j)
                    node = i*self.width + j
                    return node, coords

    def get_neighbors(self, i, j, maze_matrix):
        """Returns all neighbors for the given cell coordinates where the move is possible, and the direction of that move.

        Parameters
        ----------
        i, j : int, int
            Coordinates of the cell
        maze_matrix : []
            Matrix representation of the maze
            
        Returns
        ------
        list
            a list of neighbors in the following format: (i, j, direction)
        """
        neighbors = []
        if i-1 >= 0 and maze_matrix[i-1][j] != '#':
            neighbors.append((i-1, j, 0))
        if j+1 < self.width and maze_matrix[i][j+1] != '#':
            neighbors.append((i, j+1, 1))
        if i+1 < self.height and maze_matrix[i+1][j] != '#':
            neighbors.append((i+1, j, 2))
        if j-1 >= 0 and maze_matrix[i][j-1] != '#':
            neighbors.append((i, j-1, 3))
            
        return neighbors
    
    def fill_the_maze(self, maze_matrix):
        """Creates and returns the maze.

        Parameters
        ----------
        maze_matrix : []
            Matrix representation of the maze
            
        Returns
        ------
        dict
            a dictionary in format {node: [moves]}
        """
        maze = {}
        for i in range(self.height):
            for j in range(self.width):
                moves = []
                neighbors = self.get_neighbors(i, j, maze_matrix)
                for neigh in neighbors:
                    adj = maze_matrix[neigh[0]][neigh[1]]
                    match adj:
                        # move to exit has 0 weight
                        case 'E':
                            moves.append(Edge(neigh[0]*self.width+neigh[1], 0, neigh[2]))
                        # move to monster position is impossible
                        case 'M':
                            continue
                        # move to pathway has 10 weight
                        case '.':
                            moves.append(Edge(neigh[0]*self.width+neigh[1], 10, neigh[2]))
                maze[i*self.width+j] = moves

        # check P possible moves to get monster next possible moves and put infinite weight to moves toward the next monster position 
        neighbors = self.get_neighbors(self.init_i, self.init_j, maze_matrix)
        for neigh in neighbors:
            (p_i, p_j) = (neigh[0], neigh[1])
            (move_i, move_j) = self.monster_move(p_i, p_j, self.m_i, self.m_j, maze_matrix)
            monster_new_position = move_i*self.width+move_j
            monster_neighbors = self.get_neighbors(move_i, move_j, maze_matrix)
            for m_neigh in monster_neighbors:
                node = m_neigh[0]*self.width+m_neigh[1]
                for edge in maze[node]:
                    if edge.dest == monster_new_position and edge.dir == (m_neigh[2]+2)%4:
                        edge.weight = float("inf")
                        break
        return maze

def ucs(maze: Maze):
    """Uniform-cost search algorithm implementation.

        Parameters
        ----------
        maze : Maze
            The instance of Maze class
            
        Returns
        ------
        list
            a list of moves (shortest path) to exit
        """
    priority_queue = PriorityQueue()
    explored = set()
    sourceNode = Node(maze.init_state, 0)
    map = {}
    
    priority_queue.put(sourceNode)
    while not priority_queue.empty():
        node = priority_queue.get()
        if node.node in explored:
            continue
        if node.node == maze.goal:
            moves = []
            for i in range(len(map)):
                moves.append(map[node.node].move)
                node.node = map[node.node].node
                if node.node == maze.init_state:
                    break
            moves.reverse()
            init_edges = maze.maze[maze.init_state]
            if not moves[0] in init_edges:
                return None
            return moves
        explored.add(node.node)
        moves = maze.maze[node.node]
        for move in moves:
            child = move.dest
            cost = move.weight + node.cost
            childNode = Node(child, cost, move)
            parent = Node(node.node, cost, move)
            priority_queue.put(childNode)
            if child in map.keys():
                cost_in_map = map[child].cost
                if cost_in_map >= cost:
                    map[child] = parent
            elif not child in map.keys() and child != maze.init_state:
                map[child] = parent
    
    return None


if __name__ == "__main__":  
    ### Input from console###
    width_height = input()
    width = int(width_height.split()[0])
    height = int(width_height.split()[1])
    maze = []

    for i in range(height):
        maze_line = input()
        maze.append(maze_line)

    ### Convert input to list of lists (matrix) ###
    ### ['.', '.', '.', '.', '.', '.', '.']
    ### ['E', '#', '#', 'M', '#', '#', '.']
    ### ['.', '.', '.', '.', '.', 'P', '.']
    maze_matrix = []
    line = []
    for str in maze:
        for char in str:
            line.append(char)
        maze_matrix.append(line)
        line = []

    ### Create a maze and find a minimal path in a loop ###
    maze = Maze(maze_matrix)
    min_steps = 0
    while True:
        # Call uniform-cost search algorithm, take the first move from the path, recreate a maze
        path = ucs(maze)
        if path == None:
            print("Impossible")
            break
        else:
            move = path[0].dest
            # new position of P
            (new_p_i, new_p_j) = divmod(move, width)
            # end if P == E
            if (new_p_i, new_p_j) == (maze.g_i, maze.g_j):
                print(min_steps+1)
                break
            # move P
            maze_matrix[maze.init_i][maze.init_j] = '.' 
            maze_matrix[new_p_i][new_p_j] = 'P'
            min_steps += 1
            # recreate a maze (new state)
            maze = Maze(maze_matrix, True)
        
    