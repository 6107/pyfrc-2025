# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #
#
# Makefile Setup
#
## VERBOSE  Option - Set V (for verbose) on the command line (make V=1 <targets...>) to see additional output
ifeq ("$(origin V)", "command line")
export Q=
else
export Q=@
export MAKEFLAGS+=--no-print-directory
endif

## NO_COLOR Option - Set NO_COLOR on the command line (make NO_COLOR=1 <targets...>) to not colorize output
ifeq ("$(origin NO_COLOR)", "command line")
export GREEN  :=
export YELLOW :=
export WHITE  :=
export CYAN   :=
export RESET  :=
else
export GREEN  := $(shell tput -Txterm setaf 2)
export YELLOW := $(shell tput -Txterm setaf 3)
export WHITE  := $(shell tput -Txterm setaf 7)
export CYAN   := $(shell tput -Txterm setaf 6)
export RESET  := $(shell tput -Txterm sgr0)
endif

