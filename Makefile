# Compilador
CXX = g++

# Opciones de compilación
# Agregamos $(shell pkg-config --cflags sdl2) para obtener los headers de SDL2
CXXFLAGS = -Wall -Wextra -std=c++11 -O2 $(shell pkg-config --cflags sdl2)

# Librerías a enlazar
LIBS = $(shell pkg-config --libs sdl2)

# Nombre del ejecutable
TARGET = rc_sim

# Archivos fuente
SRC = main.cpp JoystickManager.cpp

# Archivos objeto
OBJ = $(SRC:.cpp=.o)

# Headers (para que el Makefile detecte cambios en ellos)
DEPS = JoystickManager.hpp CRSFEncoder.hpp UDPTransport.hpp

all: $(TARGET)

# Enlace del ejecutable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(TARGET) $(LIBS)

# Compilación de objetos (.o)
# Añadimos $(DEPS) para que si cambias un header, se recompile el .cpp correspondiente
%.o: %.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

debug:
	$(CXX) $(CXXFLAGS) -g $(SRC) -o $(TARGET)_debug $(LIBS)
	gdb ./$(TARGET)_debug

clean:
	rm -f $(TARGET) $(TARGET)_debug *.o

.PHONY: all run debug clean