# Hao ZHANG, 2013-3-10
# pour robot hexapode
# caesarhao@gmail.com

all: libzhrobot

libzhrobot: libzhrobot.so

libzhrobot.so: zhrobot.o
	g++ -shared -o $@ $^

%.o: %.cpp
	g++ -c -fPIC -o $@ $^

install: libzhrobot
	cp -f libzhrobot.so /usr/lib64/
	cp -f zhrobot.hpp /usr/include/

clean:
	rm *.so *.o

