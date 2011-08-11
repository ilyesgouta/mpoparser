
OBJS = mpoparser.o

%.o: %.cpp
	g++ -c -O0 -g $< -o $@

mpoparser: $(OBJS)
	g++ $< -o $@ -ljpeg

clean:
	rm -f *.o mpoparser
