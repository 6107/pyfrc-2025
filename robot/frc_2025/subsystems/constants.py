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
# Constants for source in this subdirectory will go here


# starting position for odometry
k_start_x = 0
k_start_y = 0

k_use_vision_odometry = True

k_swerve_debugging_messages = True
# multiple attempts at tags this year - TODO - use l/r/ or up/down tilted cameras again, gives better data
k_use_apriltag_odometry = True
k_use_quest_odometry = False
k_use_photontags = False  # take tags from photonvision camera
k_use_CJH_tags = True  # take tags from the pis
k_swerve_only = False
k_swerve_rate_limited = True
k_field_oriented = True
