CC := g++
CFLAGS := -Wall -Werror --std=c++17

OBJDIR := objdir
SRCDIR := src
INCDIR := include
INCS := -I/usr/include/eigen3
LIBS :=
OBJS := $(addprefix $(OBJDIR)/,estimator.o controller.o)

$(OBJDIR)/%.o : $(SRCDIR)/%.cpp $(wildcard $(INCDIR)/*/*.h)
	$(CC) -c $(CFLAGS) -I$(INCDIR) $(INCS) $< -o $@

all: $(OBJS) library

library: $(OBJS)
	$(CC) -shared -o $(OBJDIR)/libswervedrive.so $(OBJS) $(LIBS)

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	mkdir $(OBJDIR)

.PHONY: clean
clean:
	rm -Rf $(OBJDIR)
