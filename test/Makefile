TARGETS:= example_mti_receive_data
OBJLIBS	= ../include/xspublic
INCLUDE=-I. -I../include/xspublic
CFLAGS=-g $(INCLUDE)
CXXFLAGS=-std=c++11 $(CFLAGS)
LFLAGS=-L../include/xspublic/xscontroller -L../include/xspublic/xscommon -L../include/xspublic/xstypes -lxscontroller -lxscommon -lxstypes -lpthread -lrt -ldl

.PHONY: $(OBJLIBS)
all : $(OBJLIBS) $(TARGETS)

../include/xspublic :
	$(MAKE) -C ../include/xspublic $(MFLAGS)

example_mti_receive_data: example_mti_receive_data.cpp.o

$(TARGETS):
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LFLAGS)

%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean :
	-$(RM) $(OBJECTS) $(TARGETS)
	-$(RM) *.o *.d *.dpp
	-find ../include/xspublic/xscontroller -name '*.dpp' -delete
	-find ../include/xspublic/xscommon -name '*.dpp' -delete
	-find ../include/xspublic/xstypes -name '*.dpp' -delete
	-$(MAKE) -C ../include/xspublic/xscontroller $(MFLAGS) clean
	-$(MAKE) -C ../include/xspublic/xscommon $(MFLAGS) clean
	-$(MAKE) -C ../include/xspublic/xstypes $(MFLAGS) clean
