# Flipper
## Summary
`Flipper` takes in an `auto_name` from the commandline and produces a new auto called 
`auto_name (Flipped)` by taking each path in the orignal auto and transforming each of
the paths as follows:
- x' = x (no change)
- y' = L - y (reflection over L/2)
- $\theta$' = - $\theta$ (reflection over L/2)

This transformation, paired with the alliance flip already handed by PathPlanner, takes a custom-built path made in PathPlanner from the Blue Alliance and makes it usable from
the Red Alliance.

## Usage
This command is easiest to use from this directory (`scripts/flipper`):
```bash
cd scripts/flipper
javac Flipper.java && java Flipper "My Auto Name"
```
Note the quotes (`"`) are needed to handle spaces in the name.

## Cleanup
If you want to remove the generated files, run the following commands:
```bash
git restore ../../src/
rm ../../src/main/deploy/pathplanner/paths/*\(Flipped\).path 
rm ../../src/main/deploy/pathplanner/autos/*\(Flipped\).auto
```