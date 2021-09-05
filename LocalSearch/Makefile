localsearch : ls.o main.o
	g++ -o localsearch ls.o main.o -O2 -ljsoncpp

ls.o : ls.h ls.cpp
	g++ -c -Wno-deprecated-declarations ls.cpp -O2 -ljsoncpp

main.o : main.cpp ls.h
	g++ -c main.cpp -O2 -ljsoncpp

clean :
	rm localsearch ls.o main.o
