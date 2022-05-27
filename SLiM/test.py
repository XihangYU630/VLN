from ai2thor.controller import Controller

controller = Controller(
    agentMode="default",
    visibilityDistance=1.5,
    scene="FloorPlan212",

    # step sizes
    gridSize=0.25,
    snapToGrid=True,
    rotateStepDegrees=90,

    # image modalities
    renderDepthImage=False,
    renderInstanceSegmentation=False,

    # camera properties
    width=300,
    height=300,
    fieldOfView=90
)

controller.reset(scene="FloorPlan319", rotateStepDegrees=30)

controller.step("PausePhysicsAutoSim")

controller.step(
    action="AdvancePhysicsStep",
    timeStep=0.01
)


query = controller.step(
    action="GetObjectInFrame",
    x=0.64,
    y=0.40,
    checkVisible=False
)

object_id = query.metadata["actionReturn"]


controller.step(
    action="RandomizeMaterials",
    useTrainMaterials=False,
    useValMaterials=None,
    useTestMaterials=None,
    inRoomTypes=None
)

controller.step(action="RandomizeColors")

controller.step(
    action="MoveAhead",
    moveMagnitude=None
)

# Other supported directions
controller.step("MoveBack")
controller.step("MoveLeft")
controller.step("MoveRight")

controller.step(
    action="PickupObject",
    objectId="Apple|1|1|1",
    forceAction=False,
    manualInteract=False
)

event = controller.step(
    action="OpenObject",
    objectId="Book|0.25|-0.27|0.95",
    openness=1,
    forceAction=False
)
