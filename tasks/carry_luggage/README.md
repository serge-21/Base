# USER GUIDE

## Overview
This `README.md` will get you setup with the LASR container and codebase, and get you ready to contribute!

## Important
- This code will not run outside of the LASR container and the LASR work space.
- This code will not work on any OS other than Ubuntu 20.04.

## Objectives
- Setup the LASR container and your workspace.
- Create a fork of the repository.

## The Container
### Installation
If you have not already acquired the container, you will need to build it yourself. In future, you will be able to simply pull it from NextCloud.
#### Building from Source
1. Install [Apptainer](https://apptainer.org/), instructions are available [here](https://apptainer.org/docs/admin/main/installation.html).
2. Pull the *base noetic container* from NextCloud:
```bash
wget https://nextcloud.nms.kcl.ac.uk/s/BZtWJtiyP57r8YN/download -O tiago_noetic_opensource.sif -q --show-progress
```
3. Clone the [Containers](https://github.com/LASR-at-Home/Containers) repository:
```bash
git clone git@github.com:LASR-at-Home/Containers.git
```
4. Build the container, bootstrapping from the *base noetic container*:
```bash
sudo apptainer build robocup_container.sif Containers/robocup_container.def
```

### Usage
Whenever you want to use ROS, you **must** enter the container. Trying to run ROS code/commands outside of the container is a very common mistake. However, you can develop code outside of the container. You can enter the container natively as such:
```bash
apptainer run ~/robocup_container.sif
```
There are other configurations of running the container, which can be understood by running:
```bash
apptainer run-help ~/robocup_container.sif
```

### Aliases
It can be tedious to keep typing out `apptainer run...`, so it's useful and time-saving to create an alias in your `~/.bashrc`, for instance:
```bash
echo "alias robocup='sudo apptainer run ~/robocup_container.sif'" >> ~/.bashrc
source ~/.bashrc # note that this is done automatically when you launch future terminal instances.
```
## Creating a Workspace
1. Enter the container, e.g. using the alias you made above.
2. Create a workspace directory:
```bash
mkdir -p lasr_ws/src # you can call this anything, although convention is to suffix with "_ws"
```
3. Initialise the workspace:
```bash
cd lasr_ws
catkin_init_workspace
```
4. Build the workspace - although the workspace currently contains no packages, we can run a build to test that everything is setup properly.
```bash
catkin build
```
If successful, this should create `lasr_ws/build`, `lasr_ws/devel` and `lasr_ws/logs`.
## The Repository
The main LASR repository can be found [here](https://github.com/LASR-at-Home/Base), to familiarise yourself with the organisation of the repository we recommend studing the [README](https://github.com/LASR-at-Home/Base/blob/main/README.md). Understanding how things are structured will likely save you time later on when you are looking for a specific package/functionality.


### Forking the Repsository
As mentioned in the contributions guideline, working in forks is required for non-collaborative efforts (where branches would be more suited). It's likely that most work will be done in silo, and integrated with other pieces of work in a larger collaborative effort. Therefore, having a fork of the repository is essential.

1. Navigate the [repository](https://github.com/LASR-at-Home/Base).
2. Create a fork. The owner should be you, and you may want to uncheck "copy the main branch only".
3. Clone your fork:
```bash
git clone git@github.com:<username>/Base.git ~/catkin_ws/src/lasr-base
```
### Maintaining your Fork
1. Check the current remotes
```bash
git remote
```
Running this should currently output a single remote, `origin` - this relates to your own **fork**.

2. Add the base repository as a remote
```bash
git remote add base git@github.com:LASR-at-Home/Base.git
```
3. Updating your remotes - this will run `git fetch <remote>` for each remote.
```bash
git remote update
```
4. Merging from a different remote. You may want to merge/rebase the latest changes from `base/main` into/onto `<your-fork>/main`:
```bash
git checkout main # main on origin
git <rebase/merge> base/main
```
5. You can also checkout the main branch on the original repository:
```bash
git checkout base/main
```

You can also keep your branches synced through the GitHub web interface, through the `Sync fork` button

### Building
The repository ships many packages, which must be built as described above. You should now rebuild your workspace, and ensure that all packages build correctly. If you run into issues here, it is likely that you have failed to setup your environment correctly.

### Run The Code
Assuming the setup has been followed correctly, download the code and place it inside
`lasr_ws/src/laser-base/tasks` and run the following commands:
```bash
robocup
cd lasr ws
catkin build   # this will take a lot of time (approx 18 minutes)
source devel/setup.bash
roslaunch carry luggage main.launch
```
This should launch a simulation with the robot ready to execute the state machine.
