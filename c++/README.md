# Requirements

C++ 11+ and SFML (Simple and Fast Multimedia Library) is required to run the codes.

## Installation of requirements

Run the following command to install SFML on Linux (For other OS, please check the guide(s) available online)

```
$ sudo apt-get install libsfml-dev
```

## How to Build

```bash
g++ -std=c++11 -c geometry.h rrt.cpp
g++ rrt.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
```

## How to Run

```bash
./sfml-app
```
