This program tries to improve on a given solution of an instance of the CG:SHOP21 contest through local search. It can produce a solution with either a smaller makespan (MAX) or a smaller total number of moves (SUM). 

Installation:
- You need to install the json cpp library. Command under Ubuntu 20.04:
sudo apt-get install libjsoncpp-dev
- You can then compile the program by typing "make"

Syntax:
- run the program by typing 

"./localsearch instance.json solution.json MAX time time_between_solutions" 

or 

"./localsearch instance.json solution.json SUM time time_between_solutions" 

where time is the running time in seconds, and time_between_solutions is the time interval between two solutions that the program writes. It will produce solutions with name "MAX_XXX.json" or "SUM_YYY.json" where XXX is the makespan and YYY is the total number of moves.

Example:

"../localsearch small_017_20x20_90_322.instance.json solution.json MAX 600 60"
tries to improve for 10 minutes solution.json, and prints at most 10 files such as "MAX85.json", "MAX84.json" and "MAX80.json"

