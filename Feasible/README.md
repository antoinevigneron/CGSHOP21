This program constructs a feasible solution for an instance of the CG:SHOP21 contest. It can produce a solution with either a small makespan (MAX) or a small number of moves (SUM). 

Installation:
- You need to install the json cpp library. Command under Ubuntu 20.04:
sudo apt-get install libjsoncpp-dev
- You can then compile the program by typing "make"

Syntax:
- run the program by typing 

"./feasible instance.json MAX" 

or 

"./feasible instance.json SUM". 

It will produce a solution with name "MAX_XXX.json" or "SUM_YYY.json" where XXX is the makespan and YYY is the total number of moves.

