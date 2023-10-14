import bridge.processors.auxiliary as aux

##################################################
# GAME SETTING CONSTS
GK = 10
PENALTY_KICKER = 9
ENEMY_GK = 8
IS_SIMULATOR_USED = False
CAMERAS_COUNT: int = 4
MAX_BALLS_IN_CAMERA: int = 64
MAX_BALLS_IN_FIELD: int = CAMERAS_COUNT * MAX_BALLS_IN_CAMERA
BALL_PACKET_SIZE: int = 3

KEEP_BALL_DIST = 800

ROBOTS_MAX_COUNT: int = 32
TEAM_ROBOTS_MAX_COUNT: int = ROBOTS_MAX_COUNT // 2
SINGLE_ROBOT_PACKET_SIZE = 5
ROBOT_TEAM_PACKET_SIZE: int = SINGLE_ROBOT_PACKET_SIZE * TEAM_ROBOTS_MAX_COUNT

GEOMETRY_PACKET_SIZE: int = 2

DEBUG_ID = 14
DEBUG_CTRL = 14
CONTROL_MAPPING = \
{
    # DEBUG_ID: DEBUG_CTRL
    # 0: 0,
    # 1: 1,
    # 2: 2,
    # 3: 3,
    # 4: 4,
    # 5: 5,
    # 6: 6,
    # 7: 7,
    # 8: 8,
    9: 9,
    10: 10,
    11: 11,
    12: 12,
    13: 13,
    14: 14,
    None: None
}

for i in range(TEAM_ROBOTS_MAX_COUNT):
    try:
        CONTROL_MAPPING[i]
    except:
        CONTROL_MAPPING[i] = None

TOPIC_SINK = "control-sink"
##################################################

##################################################
# CONTROL CONSTS
Ts = 0.05 # s

# ROBOT SETTING CONSTS
# MAX_SPEED = 100
# MAX_SPEED_R = 50
# ACCELERATION = 3
# BASE_KICKER_VOLTAGE = 7.0
MAX_SPEED = 1500
MAX_SPEED_R = 30
SOFT_MAX_SPEED = 1000
SOFT_MAX_SPEED_R = 8
ACCELERATION = 3
BASE_KICKER_VOLTAGE = 7.0

R_KP = 7
R_KD = 0
KP = 0.1

GK_INTERCEPT_SPEED = 300
GK_PEN_KICKOUT_SPEED = 500
##################################################
# GEOMETRY CONSTS

BALL_R = 0.05
ROBOT_R = 0.2
GRAVEYARD_POS = aux.Point(-10000, 0)

POLARITY = -1
GOAL_DX = POLARITY * 4500
GOAL_DY = 1000
GOAL_PEN = POLARITY * 1000
GOAL_BOUND_OFFSET = 100
GOAL_WALLLINE_OFFSET = 1800
GOAL_WALL_ROBOT_SEPARATION = 150

GK_FORW = 800
KICK_ALIGN_DIST = 200
KICK_ALIGN_DIST_MULT = 1.5
KICK_ALIGN_ANGLE = 0.1
KICK_ALIGN_OFFSET = 20
BALL_GRABBED_DIST = 110
BALL_GRABBED_ANGLE = 0.8

# ROUTE CONSTS

VANISH_DIST = 200
