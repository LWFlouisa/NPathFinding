require_relative "lib/NeoPathfinding.rb"

matrix = [
  [1, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0]
]

grid = Grid.new(matrix)

start_node = grid.node(0, 0)
end_node = grid.node(4, 4)

finder = AStarFinder.new()
path = finder.find_path(start_node, end_node, grid)

puts grid.to_s(path, start_node, end_node)

## Analyses
evaluate_malicious_users(0.50534188, 0.49465812, 0.16, 166)
evaluate_yt_stats(:bequest_de_cendres, 166,
                  :mochitere,           87,
                  :electrorequiem,      17, 0.54)
difference_in_traffic(1, 86)

get_statistics(:github,  "There are #{1 - $malicious_humans} legitimate users, #{$malicious_humans} malicious users, and #{$malicious_bots} bots.",
               :cloaked_users, "Difference in traffic is #{$traffic_difference}.",
               :github, "There are #{1 - $malicious_humans} legitimate users, #{$malicious_humans} malicious users, and #{$malicious_bots} bots.")

dynamic_reward_allocation

word_types

sleep(1.5)
#system("clear")

matrix = [
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 0, 0, 1]
]

grid = Grid.new(matrix)

start_node = grid.node(4, 4)
end_node = grid.node(0, 0)

finder = AStarFinder.new()
path = finder.find_path(start_node, end_node, grid)

puts grid.to_s(path, start_node, end_node)

## Analyses
evaluate_malicious_users(0.50534188, 0.49465812, 0.16, 166)

evaluate_yt_stats(:bequest_de_cendres, 166,
                  :mochitere,           87,
                  :electrorequiem,      17, 0.54)
difference_in_traffic(1, 86)
      
get_statistics(:github,  "There are #{1 - $malicious_humans} legitimate users, #{$malicious_humans} malicious users, and #{$malicious_bots} bots.",
               :cloaked_users, "Difference in traffic is #{$traffic_difference}.",
               :github, "There are #{1 - $malicious_humans} legitimate users, #{$malicious_humans} malicious users, and #{$malicious_bots} bots.")

dynamic_reward_allocation

word_types
