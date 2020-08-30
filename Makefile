
# ---------------SET THESE VARIABLES-----------------
O_FOLDER = obj/
SRC_FOLDER = src/
CFLAGS = $(shell pkg-config --cflags sdl2) -g
LIBS = $(shell pkg-config --libs sdl2)
EXE_NAME = 3d_soft_render.app
# ---------------------------------------------------

CC = clang++ -std=c++20 -O3 $(CFLAGS) -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic
#CC = clang++ -std=c++20 $(CFLAGS) -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic
# CC = clang++-10 -std=c++20 $(CFLAGS) -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic

nullstring =
space = $(nullstring) #End

SRC_FILES = $(shell find $(SRC_FOLDER) -type f -name '*.cpp')
SRC_NAMES = $(patsubst $(SRC_FOLDER)%.cpp,%, $(SRC_FILES))


define MAKE_CPP
$(TARGET):				$(SRC_FOLDER)$(PRE).cpp
						$(CC) -c $(SRC_FOLDER)$(PRE).cpp -o $(SRC_FOLDER)$(PRE).o
						$(eval x=$(O_FOLDER)$(PRE).o)
						$(eval x1=$(subst /,$(space),$(x)))
						$(eval x=$(filter-out $(lastword $(x1)), $(x1)))
						$(eval x1=$(subst $(space),/,$(x)))
						mkdir -p $(x1)
						mv -f $(SRC_FOLDER)$(PRE).o $(x1)
endef

O_PATHS = $(foreach o_file, $(SRC_NAMES), $(O_FOLDER)$(o_file).o)
$(EXE_NAME):			$(O_PATHS)
						$(CC) $(O_PATHS) $(LIBS) -o $(EXE_NAME)

clean:					$(O_FOLDER)
						rm -rf $(O_FOLDER)

PRE = $(name)
TARGET = $(O_FOLDER)$(name).o

$(foreach name, $(SRC_NAMES), \
	$(eval $(MAKE_CPP)))


run:					$(EXE_NAME)
						./$(EXE_NAME)
