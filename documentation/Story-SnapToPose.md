# Snap a pose

As a driver I would like the robot to orient towards the field elements
So that angles can be picked with precision in teleop mode.



## Implementation plan
There will be a list of `fieldElements` (faces of the coral reef and the coral stations).
There will be a `buttonMapping` of controllers or controller button combos that will orient the robot.
There will be a mapping of the optimal `fieldElementOrientation` relative angle for the robot to interact with a specific element.

```java
List<String> fieldElements = new ArrayList(){"CoralStationLeft", "CoralStationRight", "ReefBottom", "ReefBottomLeft", "ReefBottomRight", "ReefTop", "ReefTopLeft", "ReefTopRight"};

Map<String, float> fieldElementOrientation = new HashMap();
fieldElementOrientation.push("CoralStationLeft", 1.0); // TODO: identify each
// TODO: push an element for each field element

Map<String, String> buttonMapping = new HashMap();
fieldElementOrientation.push("LT", "CoralStationLeft"); 
fieldElementOrientation.push("RT", "CoralStationRight"); // TODO: identify each
// TODO: push an element for each field element

```


There will use the `commands/RotateBotCommand` function `withRobotRelativeStartRads(float desiredAngle)` that takes an angle as a desired angle as arugment and will orient the robot to that position.  This will compare the robot's currentAngle currentPose and will calculate a movement to change from current to desired.

There will be a function `void orientTowardsElement(String fieldElementId)` that takes a named element of the field.
The controller button pushed will look up the element it is supposed to orient towards and will call the `orientTowardsElement`
Pressing the button twice won't change the robots orientation.
Holding the button won't change the orientation and will try to maintain that orientation as it moves.



Possible optimizations
Will this work when driving?
Will we need to set orientation based on team?

There will be 6 orientations programmed related to the faces of the coral reef

There will be 2 orientations programmed related to the coral stations

When a 
