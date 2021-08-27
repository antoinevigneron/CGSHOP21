This program tries to improve on a given solution of an instance of the CG:SHOP21 contest through simulated annealing. It can produce a solution with either a smaller makespan (MAX) or a smaller total number of moves (SUM). 

INSTALLATION

- You need to install the json cpp library. Command under Ubuntu 20.04:
sudo apt-get install libjsoncpp-dev
- You can then compile the program by typing "make"

SYNTAX

- run the program by typing 

"./sa instance.json solution.json MAX" 

or 

"./sa instance.json solution.json SUM" 

It will produce solutions with name "MAX_XXX.json" or "SUM_YYY.json" where XXX is the makespan and YYY is the total number of moves.

EXAMPLE

"../ls small_017_20x20_90_322.instance.json solution.json MAX"
tries to improve the makespan of solution.json 

PARAMETERS

By default, this program does simulated annealing using the parameters written in the files defaultMAX.par or defaultSUM.par for the MAX or the SUM version, respectively. It is also possible to read the parameters from another file. For instance:

"./sa instance.json solution.json MAX mypar.par" 

uses the parameters from the file mypar.par. 

Several parameters can modified in this file. The most useful are the number of iterations per cooling cycle, and the number of cooling cycles. Increasing these parameters will make the program run for longer and hopefully produce a better solution. 

One can also modify the maximum temperature Tmax. For large instances, a smaller temperature may help: If the temperature is too high, the solution tends to drift away and the score keeps getting worse.

The frequency of stretching and tightening moves can also be adjusted. For instance, if we set 7 for stretching and 3 for tightening, then the algorithms has 70% chance of generating a stretching move and 30% chance generating a tightening move.

A different integer random seed can also be specified so that the algorithm may produce a different result.



