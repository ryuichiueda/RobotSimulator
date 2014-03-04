CXX = g++
TARGET = main
CXXFLAGS = -Wall -O3 --static
LDFLAGS = -lm
SRCS := $(wildcard *.cpp)
OBJS := $(SRCS:.cpp=.o) #SRCSの各ファイルのサフィックスの.cppを.oに変換

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) 

clean:
	rm -f $(TARGET) $(OBJS)
