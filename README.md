# Search Algorithm

This program is to test the search algorithm between Breadth First Search, Depth First Search, A* and Iterative Deepening A* with a 3x3 matrix containing value 0-8.

To run the program, use command line: python search.py a b

a could be: bfs dfs ast ida

b is a list containing value 0-8

Example
to run the program using A* with this matrix

0 2 5

3 4 1

6 7 8

run:
python search.py ast 0,2,5,3,4,1,6,7,8
The program will move 0 around and save the result in output.txt

THe program will keep runing even if there is no solution of the matrix
