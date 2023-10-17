from importlib.resources import path
from base import (BaseAgent, action_dict, move,
                  set_timeout, after_timeout, TIMEOUT)

##################################################################################
# Here is a demo agent.                                                          #
# You can implement any helper functions.                                        #
# You must not remove the set_timeout restriction.                               #
# You can start your code any where but only the get_action() will be evaluated. #
##################################################################################


class MyAgent(BaseAgent):

    def get_avai_actions(self, game_state):
        avai_actions = []
        for action in action_dict:
            fake_action_profile = dict()
            for name in game_state:
                if name == self.name:
                    fake_action_profile[name] = action
                else:
                    fake_action_profile[name] = 'nil'
            succ_state = self.env.transition(game_state, fake_action_profile)
            if succ_state:
                avai_actions.append(action)
        return avai_actions

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        #print("inside reconstruct ", end = "")
        #print(total_path)
        total_path.reverse()
        return total_path

    def Astar(self, goal, game_state, check = []):
        obs = self.observe(game_state)
        initial_state = obs[0]
        step_cost = 1

        closed_list = set()
        open_list = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = heuristic_manhattan(goal, initial_state)

        while open_list:
            #print(f'open list = {open_list}')
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_list}
            current = min(temp_dict, key=temp_dict.get)
            #print("current vs goal", end = " = ")
            #print(current, end = " vs ")
            #print(goal)
            if current == goal:
                #print("goal reached in astar")
                #print(f'current game state = {game_state}')
                #print("recovering game_state")
                game_state[self.name] = initial_state
                #print(f'current game state = {game_state}')
                return self.reconstruct_path(came_from, current)
            
            open_list -= {current}
            closed_list |= {current}
            #print(f'closed list = {closed_list}')
        
            #print(game_state)
            game_state[self.name] = current
            avail_actions = self.get_avai_actions(game_state)
            #print(f'updated game_state = {game_state}')

            neighbour_list = []
            for action in avail_actions:
                if check == []:
                    if (action == 'nil'):
                        continue
                    obs = self.observe(game_state)
                    succ = move(obs[0], action)
                    neighbour_list.append(succ)
                else:
                    #print("constraint checking")
                    if (action == 'nil'):
                        continue
                    obs = self.observe(game_state)
                    succ = move(obs[0], action)
                    if succ != check[1] and succ != check[0]:
                        neighbour_list.append(succ)


            #print(f'neighbour =  {neighbour_list}')

            for neighbour in neighbour_list:
                if neighbour in closed_list:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbour not in open_list:
                    #print(f'neighbour {neighbour} added')
                    open_list |= {neighbour}
                elif tentative_g_score >= g_score.setdefault(neighbour, float("inf")):
                    continue

                came_from[neighbour] = current
                g_score[neighbour] = tentative_g_score
                f_score[neighbour] = g_score[neighbour] + heuristic_manhattan(goal, neighbour)

    @set_timeout(TIMEOUT, after_timeout)
    def get_action(self, game_state):
        # Step 1. figure out what is accessible
        obs = self.observe(game_state)
        avai_actions = self.get_avai_actions(game_state)
        goal = self.env.get_goals()[self.name]
        #trial
        keys = list(game_state.keys())
        second_agent = []
        for key in keys:
            if self.name == key:
                continue
            else:   
                second_agent.append(key)
                
        second_goal = self.env.get_goals()[second_agent[0]]
        # Step 2. production system or any rule-based system
        
        sol1 = self.Astar(goal, game_state)
        #print(f'sol1 = {sol1}')
        temp_game_state = {}
        second_loc = obs[1]
        temp_game_state[self.name] = second_loc
        temp_game_state[second_agent[0]] = obs[0]
        sol2 = self.Astar(second_goal, temp_game_state)
        #print(f'sol2 = {sol2}')
        #determine which method of A star we using
        #check = self.constraint(second_agent, sol1, sol2)
        if len(sol2) > 1:
            check = sol2
        else: 
            check = []
        solution = self.Astar(goal, game_state, check)

        #print(f'after Astar game state = {game_state}')
        #print(f'orig solution = {solution}')
        #print(f'solution = {solution}')

        target = solution[1]
        #print(f'target: {target}')
        for action in avai_actions:
            succ = move(obs[0], action)
            if succ in obs[1:]:
                continue
            else:
                if succ == target:
                    return action
        
def heuristic_manhattan(goal, pos) -> float:
    return abs(goal[0] - pos[0]) + abs(goal[1] - pos[1])
