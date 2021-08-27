sa : sa.o main.o
	g++ -o sa sa.o main.o -O2 -ljsoncpp

sa.o : sa.h sa.cpp
	g++ -c -Wno-deprecated-declarations sa.cpp -O2 -ljsoncpp

main.o : main.cpp sa.h
	g++ -c main.cpp -O2 -ljsoncpp

clean : 
	rm sa sa.o main.o
