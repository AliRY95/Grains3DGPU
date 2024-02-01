objects = Grains.o Convex.o Vector3.o 

all: $(objects)
	nvcc -arch=sm_60 $(objects) -o grains -lcudart

%.o: %.cpp
	nvcc -x cu -arch=sm_60 -I. -dc $< -o $@

clean:
	rm -f *.o grains
