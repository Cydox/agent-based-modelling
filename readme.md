# AE4420-20 Assignment code base

This is the code base for the assignment for the AE4420-20 coarse in the aerospace engineering master.

Authors: Jan Hendrik Farr and Vince van Deursen

### Code base structure:
* Instances folder contains instances (i.e. cases) for a simulation to be run. 
* Original code folder contains original code. 
* results_<em>solver</em> folder contains all the results files for the cases run for <em>solver</em>.


### Python scripts: 
There are two scripts that will run simulations. 
* `run_experiments.py`: performs one simulation for predefined agent start and goal locations. 
* `run_orchestrator_multi.py`: performs simulations for a certain agent group configuration until of coefficient of variation stabilizes.


### Instance types:
There are two types of instances: 
* The 'original' instances defined separate agents in each line with each agent a predefined start group. This type of instances is loaded by the `run_experiments.py` script.
* The `run_orchestrator_multi.py` takes the other instance type. With this type, agent groups are described in the map. See description of `import_mapf_instance()` function in `library_open_simulations.py` for detailed explanation.


### Description of (self created) python files:
| File | Description
|:----:|------|
|`orchestrate_simulations.py`  |    Contains the `Case` and `Orchestrator` classes which are being parallelized by `run_orchestrator_multi.py` script to use multi-processing while running multiple cases. |
|`distributed.py`| Contains the `DistributedSolver` class which implements the Distributed planner model. This file actually serves as the 'environment' to the agents.|
|`distributed_agent.py`| Contains the `AgentDistributed` class which are instantiated in the Distributed planner model. |
|`library_open_simulation_config.py` |  This file contains functions that load the revised instance type.  |
|`create_assignment_files.py` |  Not that important. Just a helper function to automatically build the insta nce files.|

### Description of other files: 

| File | Description
|:----:|------|
|`orchestrate_simulations.py`  |    Contains the `Case` and `Orchestrator` classes which are being parallelized by `run_orchestrator_multi.py` script to use multi-processing while running multiple cases. |
|`distributed.py`| Contains the `DistributedSolver` class which implements the Distributed planner model. This file actually serves as the 'environment' to the agents.|
