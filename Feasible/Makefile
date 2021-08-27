feasible : feasible.o main.o
	g++ -o feasible feasible.o main.o -O2 -ljsoncpp

feasible.o : feasible.h feasible.cpp
	g++ -c -Wno-deprecated-declarations feasible.cpp -O2 -ljsoncpp

main.o : main.cpp feasible.h
	g++ -c main.cpp -O2 -ljsoncpp

clean :
	rm feasible feasible.o main.o
