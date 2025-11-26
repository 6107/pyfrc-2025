#!/usr/bin/env bash
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
# load local python virtualenv if exists
VENVDIR=${VENVDIR:-venv}
PACKAGEDIR=${PACKAGEDIR:-robot}
PYVERSION=${PYVERSION:-"3.13"}

cd ${PACKAGEDIR}
if [ -e "${VENVDIR}/.built" ]; then
    . $VENVDIR/bin/activate
else
   echo "Creating python development environment"
 	 python3 -m virtualenv --python=python${PYVERSION} -v ${VENVDIR} &&\
        source ./${VENVDIR}/bin/activate && set -u && \
        pip install --disable-pip-version-check -r requirements.txt && \
        uname -s > ${VENVDIR}/.built
fi
