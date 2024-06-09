# Variáveis
CXX = g++
CXXFLAGS = -Iinclude -g -Wall -std=c++11
LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system

# Arquivos de entrada e saída
TARGET = rrt-app
SOURCES = src/rrt.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Regras
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJECTS)

# Force build
fb: clean all

run:
	./rrt-app