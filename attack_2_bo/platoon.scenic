#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('./../Town06.xodr')
param carla_map = 'Town06'
param time_step = 1.0/10
model scenic.simulators.lgsvl.model
param render = True
param verifaiSamplerType = 'bo'

# Parameters of the scenario.
param EGO_SPEED = VerifaiRange(2, 30)
param EGO_BRAKING_THRESHOLD = VerifaiRange(5, 15)

#CONSTANTS
TERMINATE_TIME = 40 / globalParameters.time_step
CAR3_SPEED = 20
CAR4_SPEED = 20
LEAD_CAR_SPEED = 20
MODEL = "vehicle.tesla.model3"
BRAKE_ACTION = 1.0
THROTTLE_ACTION = 0.6


LEADCAR_TO_EGO = 7
EGO_TO_C3 = 7
C3_TO_C4 = 7
SPAWN = 7

C3_BRAKING_THRESHOLD = 6
C4_BRAKING_THRESHOLD = 6
LEADCAR_BRAKING_THRESHOLD = 6


## DEFINING BEHAVIORS
#COLLISION AVOIDANCE BEHAVIOR
behavior CollisionAvoidance(safety_distance=10):
	take SetBrakeAction(BRAKE_ACTION)

#EGO BEHAVIOR: Follow lane, and brake after passing a threshold distance to the leading car
behavior EgoBehavior(speed=10):

	try:
		do FollowLaneBehavior(speed)

	interrupt when withinDistanceToAnyObjs(self, globalParameters.EGO_BRAKING_THRESHOLD):
		do CollisionAvoidance(globalParameters.EGO_BRAKING_THRESHOLD)

#LEAD CAR BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior LeadingCarBehavior(speed=10):

	try:
		do FollowLaneBehavior(speed)

	interrupt when withinDistanceToAnyObjs(self, LEADCAR_BRAKING_THRESHOLD):
		do CollisionAvoidance(LEADCAR_BRAKING_THRESHOLD)

#CAR3 BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior Car3Behavior(speed=10):

	try:
		do FollowLaneBehavior(speed)

	interrupt when withinDistanceToAnyObjs(self, C3_BRAKING_THRESHOLD):
		do CollisionAvoidance(C3_BRAKING_THRESHOLD)

#CAR4 BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior Car4Behavior(speed=10):

	try:
		do FollowLaneBehavior(speed)

	interrupt when withinDistanceToAnyObjs(self, C4_BRAKING_THRESHOLD):
		do CollisionAvoidance(C4_BRAKING_THRESHOLD)

#PLACEMENT
initLane = network.roads[0].forwardLanes.lanes[0]
spawnPt = initLane.centerline.pointAlongBy(SPAWN)


ego = Car following roadDirection from spawnPt for EGO_TO_C3,
		with blueprint MODEL,
		with behavior EgoBehavior(globalParameters.EGO_SPEED)

leadCar = Car following roadDirection from ego for LEADCAR_TO_EGO,
    with behavior LeadingCarBehavior(LEAD_CAR_SPEED),
	with blueprint MODEL

#require always (distance from ego.position to c3.position) > 4.99
require always (distance from ego.position to leadCar.position) > 4.99
terminate when leadCar.lane == None
terminate when simulation().currentTime > TERMINATE_TIME
