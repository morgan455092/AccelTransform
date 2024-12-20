TARGETS:= example_mti_receive_data
OBJLIBS	= include/xspublic
INCLUDE=-I. -Iinclude/xspublic
CFLAGS=-g $(INCLUDE)
CXXFLAGS=-std=c++11 $(CFLAGS)
LFLAGS=-Lxspublic/xscontroller -Lxspublic/xscommon -Lxspublic/xstypes -lxscontroller -lxscommon -lxstypes -lpthread -lrt -ldl

.PHONY: $(OBJLIBS)
all : $(OBJLIBS) $(TARGETS)

xspublic :
	$(MAKE) -C xspublic $(MFLAGS)

example_mti_receive_data: example_mti_receive_data.cpp.o

$(TARGETS):
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LFLAGS)

%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean :
	-$(RM) $(OBJECTS) $(TARGETS)
	-$(RM) *.o *.d *.dpp
	-find include/xspublic/xscontroller -name '*.dpp' -delete
	-find include/xspublic/xscommon -name '*.dpp' -delete
	-find include/xspublic/xstypes -name '*.dpp' -delete
	-$(MAKE) -C include/xspublic/xscontroller $(MFLAGS) clean
	-$(MAKE) -C include/xspublic/xscommon $(MFLAGS) clean
	-$(MAKE) -C include/xspublic/xstypes $(MFLAGS) clean