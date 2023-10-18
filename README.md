# Project Description
The task of this project is to design an environment-specific controller for two-agent cases. To achieve this, a method of Conflict-based Search (CBS) combined with A* Search is used.

## Methodology
The details of the CBS A* Search will be discussed here. The idea is to create a two-level algorithm – high level and low level. The low level looks at each individual agent and finds the optimal route from a given state to the goal, while the high level makes sure that there will be no conflicts by looking into the constraints of each agent at a particular state.


## Low-Level A* Search
`Astar(self, goal, game_state)`
The low level of the algorithm utilizes A* Search to search for the shortest path for a given state to a goal. The procedures inside are tantamount to typical A*star algorithms, but the heuristic function shall be a vital point to note. In this implemented A* Search, Manhattan distance is used as the heuristic, where the cost for each move is set to be 1. This decision is made based on the 4-direction nature of the robots.
`(heuristic_manhattan(goal, pos) -> float)`

## High-Level A* Search
`Astar(self, goal, game_state, check = constraint_list)`
The High-Level algorithm is the key to being a Conflict-Based Search. The only difference between the high-level and the low-level is the addition of a list which gives information about the constraints of an agent at a particular game state. The constraint list is generated by considering the state-specific possible actions of another agent. The high-level will utilize this information to generate a conflict-free solution. In particular, it aims to perform a best-first search on the available actions according to the costs.


### Dependence:

```
pip install -r requirements.txt
```

### Usage:

```
usage: run.py [-h] [--agents AGENTS [AGENTS ...]] [--map MAP] [--goals GOALS [GOALS ...]] [--vis]
              [--save SAVE]

Multi-Agent Path Finding Term Project.

optional arguments:
  -h, --help            show this help message and exit
  --agents AGENTS [AGENTS ...]
                        Specify a list of agent names
  --map MAP             Specify a map
  --goals GOALS [GOALS ...]
                        Specify the goals for each agent,e.g. 2_0 0_2
  --vis                 Visulize the process
  --save SAVE           Specify the path to save the animation video
```

Example:

```
python run.py --agents p1 p2 --map empty --goals 5_5 1_5 --vis --save empty.mp4
```

You may need to install `ffmpeg` to support `.mp4` extension,

```
conda install -c conda-forge ffmpeg
```
