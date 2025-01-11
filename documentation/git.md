# Git 
## Cloning this Repository
```bash
git clone https://github.com/frc4087/SwerveDriveBase2025.git
```

## Creating a new Branch
```bash
git checkout -b <your branch name>
```

## Looking at Your Commit History
```bash
git log
```
Each block in this output is a commit.

## Creating a Commit
Creating a commit requires three things:
1. Adding changes
2. Commiting those changes
3. Pushing those changes to origin (GitHub)
```bash
git add . 
git commit -m "<your commit message>"
git push
```
Keep your commit messages short and descriptive. When you push for the first time,
you'll see a message about having no upstream branch. To set up the origin branch
to automatically use the name of your local branch, run the following command:
```bash
git config --global --add --bool push.autoSetupRemote true
```

## Example Workflow
Create a branch when you're working on a chunk of code that you'd like reviewed as a unit.
This code will end up on the `main` branch after review, letting you revert and reference it
later. In this example, we'll be creating an initial draft of the Autonomous mode functionality.

1. Create a branch with a decsriptive name (ex. `initial-auto-mode`).
    ```bash
    git checkout -b initial-auto-mode
    ```
2. Create commits as you go. You can have many commits on a branch. Remember: commits are like checkpoints.
    ```bash
    git add .
    git commit -m "first step"
    ```
3. When you're ready to have the code reviewed or want others to be able to look at your code, push the code to the origin (GitHub)
    ```bash
    git push
    ```
4. At this point, you code can be seen and used by other people. People can get the code locally by "pulling" your branch:
    ```bash
    git pull origin/initial-auto-mode
    ```
5. When you're ready to have your code reviewed, [create a "Pull Request" (PR)](https://github.com/frc4087/SwerveDriveBase2025/compare). Select your branch name (ex. `initial-auto-mode`) as the `compare` branch and `main` as the base branch. 
6. You can common on specific section of code or on the PR as a whole.
7. When the code is in a state that could be put in the robot and review comments have been addressed, merge the PR.
8. To "refresh" your local machines view, you need to `checkout` the `main` branch then `pull` the code.
    ```bash
    git checkout main
    git pull
    ```