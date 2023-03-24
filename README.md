# AI-course---task-1---search
### Q1
This is just the algorithm learned in class. 

### Q2
This is just the algorithm learned in class. 

### Q3
This is just the algorithm learned in class. 

### Q4
This is just the algorithm learned in class. 

### Q5
In `CornersProblem`, the state we chose is of the form `(pacmans x,y position), (remaining corners list...)`.
This way when eating a corner we just remove it from the list,
and a goal state is reached when the corners list is empty.

### Q6
The heuristic we chose for `cornersHeuristic` is such: 
sum the minimal manhattan distances from pacman to the closest corner, and from this corner to the other closest corner, etc, until summing all corners (in `find_closest_corner`).
<br>
This essentially gives the "manhattan" distance of a path going from pacman to closest corner, and then going only on the perimeter of the board between the corner.
This can be seen to be a consistent heuristic, noting (after some thought) it is the real minimal path for the relaxed problem of the same board without any walls.
### Q7
The heuristic we chose for `AStarFoodSearchAgent` (in `foodHeuristic`)
is the one which returns the length of the longest valid path from pacman to any food on the board (`get_farthest_food_distance_with_grid`).
<br>
First of all, this can be seen to be a consistent heuristic:
since the cost of moving from state n to a neighbour n' has cost(n,n',a) = 1 (for any valid action a),
the cost of the longest path after the move cannot decrease by more then 1.
Hence the inequality h(n) <= h(n') + cost(n,n', a) holds.  

To find this longest path, starting from each iteration of pacman position, we scan the board in a bls fashion,
moving from each position to its neighbors,
and writing in each location the (minimal) path distance to this location (in `generate_distance_grid`).
<br>
Finally, going over all existing foods, we chose the one with maximal value in the generated green (in `distances_grid`),
and this is longest path from pacman to any existing food.
<br>
Remark: we found this method to be more efficient compared to running the already implemented bfs search problem from pacman to each food. 

### Q8
To find the actions leading pacman to the closest food (in `ClosestDotSearchAgent.findPathToClosestDot`),
we've just returned the actions of the bfs search result on a problem where *any* food is the goal (in `AnyFoodSearchProblem.isGoalState`).
<br> By the nature of bfs, the first goal reaches is (one of) the closest to pacman, hence returns what's needed.
