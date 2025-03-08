# Internet Connection
Competition areas do not have reliable internet connection. To prepare, 
- [ ] Verify all software (versions, tools)
- [ ] Pull `main` on each of the competition laptops
- [ ] Build the project on each of the competition laptops

The [GradleRIO](https://github.com/wpilibsuite/GradleRIO?tab=readme-ov-file#tools) plugin has a `--offline` flag for certain commands. From their documentation:
> **At Competition? Connected to the Robot?** Run with the `--offline` flag. e.g. `./gradlew deploy --offline`

# Reverting Changes
## Local Changes
If you've made changes locally that you want to remove, use `git restore`:
```bash
# Undo all changes
git restore .

# Undo changes in a directory
git restore src/main/java/frc/robot/subsystems/

# Undo changes to a file
git restore src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
```
## Rollback to an Earlier Commit
There are two ways to undo committed changes:
1. Reverting (think `control+z`)
2. Checkout (think "Take me here")
**If you're in a rush during competition, use checkout and figure out the rest later.**
### Reverting
If you want to remove a commit, use git revert:
```bash
# Undo latest commit
git revert HEAD

# Undo commit by git SHA
git revert 874a692d9a339f8d7ba7e643d3a3576dffcdeadf 
```
If you're reverting multiple commits, make sure to revert from most-recent to least-recent. For example, commits `A` -->  `B` --> `C` should be reverted in the following order:
```bash
git revert C
git revert B
git revert A
```
### Checking Out
To use a previously-committed version of the project, you need the git SHA of that commit. You can see it in GitHub or by running `git log`. In the example below,
```bash
❯ git log -n 2
commit 874a692d9a339f8d7ba7e643d3a3576dffcdeadf (HEAD -> main, origin/main, origin/HEAD)
Author: Allen <rallensanford@pm.me>
Date:   Sat Mar 8 12:17:20 2025 -0600

    Remove unused imports and update GradleRIO

commit c5185a4d58e3cd2b2045908b1248ef754223e0dd
Author: Allen <57648478+san4d@users.noreply.github.com>
Date:   Sat Mar 8 10:30:10 2025 -0600

    Sam elastic (#30)
    
    * added elastic to play with layouts
    
    * added v_batt & arm_pos to dash
    
    * saved elastic layout json to deploy directory; can download from code nowh
    
    ---------
    
    Co-authored-by: Sam Woodham <samwoodham@Sams-Laptop.local>
```
the most-recent commit is `874a692d9a339f8d7ba7e643d3a3576dffcdeadf` and the commit before that is `c5185a4d58e3cd2b2045908b1248ef754223e0dd`. 

To checkout a specific commit:
```bash
# General
git checkout <git SHA>

# From example
git checkout c5185a4d58e3cd2b2045908b1248ef754223e0dd
```
