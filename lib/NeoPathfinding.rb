# frozen_string_literal: true

# frozen_string_literal: true

require_relative "NeoPathfinding/version"

#module NeoPathfinding
  #class Error < StandardError; end
  # Your code goes here...
#end

class AStarFinder
  #
  # Initializes the A* path-finder. Params:
  # * +heuristic+: heuristic function (see the +Heuristic+ module)
  # * +diagonal_movement+: set diagonal movements (see the +DiagonalMovement+ module)
  #
  def initialize(
    heuristic = Heuristic::method(:manhattan),
    diagonal_movement = DiagonalMovement::NEVER
  )
    @diagonal_movement = diagonal_movement
    if diagonal_movement == DiagonalMovement::NEVER
      @heuristic = heuristic
    else
      @heuristic = Heuristic::method(:octile)
    end
  end

  #
  # Finds and returns the path as a list of node objects.
  #
  def find_path(start_node, end_node, grid)
    open_set = [start_node]
    came_from = {}
    g_score = {}
    f_score = {}
    grid.each_node do |node|
      g_score[node] = Float::INFINITY
      f_score[node] = Float::INFINITY
    end
    g_score[start_node] = 0
    f_score[start_node] = @heuristic.call(
      (start_node.x - end_node.x).abs, (start_node.y - end_node.y).abs)

    until open_set.empty?
      current = open_set[0]
      open_set.each do |node|
        current = node if f_score[node] < f_score[current]
      end

      if current == end_node
        return reconstruct_path(came_from, current)
      end

      current = open_set.delete_at(open_set.index(current))

      grid.neighbors(current, @diagonal_movement).each do |neighbor|
        tentative_g_score = g_score[current] + d(current, neighbor)
        next if tentative_g_score >= g_score[neighbor]

        came_from[neighbor] = current
        g_score[neighbor] = tentative_g_score
        f_score[neighbor] = g_score[neighbor] + @heuristic.call(
          (neighbor.x - end_node.x).abs, (neighbor.y - end_node.y).abs)
        unless open_set.include?(neighbor)
          open_set << neighbor
        end
      end
    end
  end

  #
  # Returns the distance between two nodes.
  #
  def d(n1, n2)
    (n1.x == n2.x || n1.y == n2.y) ? 1 : Math.sqrt(2)
  end

  #
  # Reconstructs the path from the current node.
  #
  def reconstruct_path(came_from, current)
    total_path = [current]
    while came_from.include?(current)
      current = came_from[current]
      total_path << current
    end
    total_path.reverse
  end
end

module DiagonalMovement
  # Always accept diagonal movements.
  ALWAYS = 1

  # Never accept diagonal movements.
  NEVER = 2

  # Accept a diagonal movement if there is at most one obstacle.
  IF_AT_MOST_ONE_OBSTACLE = 3

  # Accept a diagonal movement only when there is no obstacle.
  ONLY_WHEN_NO_OBSTACLE = 4
end

class Grid
  include Enumerable

  #
  # Creates a grid from a matrix:
  # * 0 (or less) represents a walkable node
  # * A number greater than 0 does not represents a walkable node
  # The +width+ represents the number of columns whereas the +height+
  # is the number of rows of the grid. The +node+ attribute
  # is the list of nodes.
  #
  def initialize(matrix)
    @height = matrix.length
    @width = matrix[0].length
    @nodes = Grid.build_nodes(@width, @height, matrix)
  end

  #
  # Gets the node at position (+x+, +y+).
  #
  def node(x, y)
    @nodes[y][x]
  end

  #
  # Yields all nodes of the grid.
  #
  def each_node
    @height.times do |y|
      @width.times do |x|
        yield node(x, y)
      end
    end
  end

  #
  # Creates a printable string from the grid using ASCII characters.
  # Params:
  # +path+:: list of nodes that show the path
  # +start_node+:: start node
  # +end_node+:: end node
  # +border+:: create a border around the grid
  # +start_chr+:: character for the start (default "s")
  # +end_chr+:: character for the end (default "e")
  # +path_chr+:: character for the path (default "x")
  # +empty_chr+:: character for the empty fields (default " ")
  # +block_chr+:: character for the blocking elements (default "#")
  #
  def to_s(
    path = nil, start_node = nil, end_node = nil, border = true,
    start_chr = 's', end_chr = 'e', path_chr = 'x', empty_chr = ' ', block_chr = '#'
  )
    data = []
    data << '+' + '-' * @width + '+' if border
    @height.times do |y|
      line = ''
      line += '|' if border
      @width.times do |x|
        current = node(x, y)
        if current == start_node
          line += start_chr
        elsif current == end_node
          line += end_chr
        elsif path&.include?(current)
          line += path_chr
        elsif current.walkable
          line += empty_chr
        else
          line += block_chr
        end
      end
      line += '|' if border
      data << line
    end
    data << '+' + '-' * @width + '+' if border
    data.join("\n")
  end

  #
  # Returns if the (+x+, +y+) position is in the grid.
  #
  def inside?(x, y)
    x >= 0 && x < @width && y >= 0 && y < @height
  end

  #
  # Returns if a node at position (+x+, +y+) is walkable.
  #
  def walkable?(x, y)
    inside?(x, y) && node(x, y).walkable
  end

  #
  # Get all neighbors of a node.
  #
  def neighbors(node, diagonal_movement=DiagonalMovement::NEVER)
    x = node.x
    y = node.y
    neighbors = []
    s0 = d0 = s1 = d1 = s2 = d2 = s3 = d3 = false

    # ↑
    if walkable?(x, y - 1)
      neighbors << node(x, y - 1)
      s0 = true
    end

    # →
    if walkable?(x + 1, y)
      neighbors << node(x + 1, y)
      s1 = true
    end

    # ↓
    if walkable?(x, y + 1)
      neighbors << node(x, y + 1)
      s2 = true
    end

    # ←
    if walkable?(x - 1, y)
      neighbors << node(x - 1, y)
      s3 = true
    end

    return neighbors if diagonal_movement == DiagonalMovement::NEVER

    if diagonal_movement == DiagonalMovement::ONLY_WHEN_NO_OBSTACLE
      d0 = s3 && s0
      d1 = s0 && s1
      d2 = s1 && s2
      d3 = s2 && s3
    elsif diagonal_movement == DiagonalMovement::IF_AT_MOST_ONE_OBSTACLE
      d0 = s3 || s0
      d1 = s0 || s1
      d2 = s1 || s2
      d3 = s2 || s3
    elsif diagonal_movement == DiagonalMovement::ALWAYS
      d0 = d1 = d2 = d3 = true
    else
      raise 'Incorrect value of diagonal_movement'
    end

    # ↖
    neighbors << node(x - 1, y - 1) if d0 && walkable?(x - 1, y - 1)

    # ↗
    neighbors << node(x + 1, y - 1) if d1 && walkable?(x + 1, y - 1)

    # ↘
    neighbors << node(x + 1, y + 1) if d2 && walkable?(x + 1, y + 1)

    # ↙
    neighbors << node(x - 1, y + 1) if d3 && walkable?(x - 1, y + 1)

    neighbors
  end

  #
  # Builds and returns the nodes.
  #
  def self.build_nodes(width, height, matrix)
    nodes = []
    height.times do |y|
      nodes << []
      width.times do |x|
        walkable = matrix[y][x] <= 0
        nodes[y] << Node.new(x, y, walkable)
      end
    end
    nodes
  end
end

module Heuristic
  #
  # Manhattan distance.
  #
  def self.manhattan(dx, dy)
    dx + dy
  end

  #
  # Euclidean distance.
  #
  def self.euclidean(dx, dy)
    Math.sqrt(dx * dx + dy * dy)
  end

  #
  # Octile distance.
  #
  def self.octile(dx, dy)
    f = Math.sqrt(2) - 1
    dx < dy ? f * dx + dy : f * dy + dx
  end

  #
  # Chebyshev distance.
  #
  def self.chebyshev(dx, dy)
    [dx, dy].max
  end
end

class Node
  # Gets the x coordinate in the grid.
  attr_reader :x

  # Gets the y coordinate in the grid.
  attr_reader :y

  # Gets whether the node is walkable.
  attr_reader :walkable

  #
  # Creates a node.
  #
  def initialize(x, y, walkable = true)
    @x = x
    @y = y
    @walkable = walkable
  end

  #
  # Makes the string format of a node.
  #
  def to_s
    "(#{@x}, #{@y})"
  end
end

def get_statistics(a1, a2, b1, b2, c1, c2)
  a = a1, a2
  b = b1, b2
  c = c1, c2

  matrix = [
    [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
    [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
    [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
  ], [
    [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
    [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
    [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
  ], [
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
  ]

  label_type       = matrix[0]
  definition_type  = matrix[1]
  probability_type = matrix[2]
  
  row_probability = 0.33
  col_probability = 0.33
  
  graph_selection = row_probability * col_probability

  row_options = [0, 1, 2]
  col_options = [0, 1, 2]
  arr_options = [0, 1]

  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample
  
  current_label       = label_type[cur_row][cur_col][cur_arr]
  current_definition  = definition_type[cur_row][cur_col][cur_arr]
  current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
  puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
  $current_probability = current_probability + current_probability
  $current_information = "#{current_label} #{current_definition}"
  
  puts "---"
end

def get_statistics_version2(a1, a2, b1, b2, c1, c2)
  ## The initial probability of such an outcome for any given higher order set is smaller: 0.035937

  a = "#{a1} #{a2}"
  b = "#{b1} #{b2}"
  c = "#{c1} #{c2}"
  
  dataset = [
    [[a, a, a], [a, a, b], [a, a, c]],
    [[b, b, a], [b, b, b], [b, b, c]],
    [[c, c, a], [c, c, b], [c, c, c]],
  ], [
    [[0.33, 0.33, 0.33], [0.33, 0.33, 0.33], [0.33, 0.33, 0.33]],
    [[0.33, 0.33, 0.33], [0.33, 0.33, 0.33], [0.33, 0.33, 0.33]],
    [[0.33, 0.33, 0.33], [0.33, 0.33, 0.33], [0.33, 0.33, 0.33]],
  ]
  
  # The probability for rows and col locations for any given 3D graph.
  row_probability = 0.33
  col_probability = 0.33
  
  factoid_information = dataset[0]
  factoid_probability = dataset[1]
  
  row_options = [0, 1, 2]
  col_options = [0, 1, 2]
  arr_options = [0, 1, 2]
  
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample
  
  @current_factoid = factoid_information[cur_row][cur_col][cur_arr]
  
  selection_probability           = factoid_probability[cur_row][cur_col][cur_arr]
  @final_probability_determination = row_probability * col_probability * selection_probability
end

def reasses
  if $current_probability > 0.999999999999999999
    $current_probability = 0.9 / $current_probability
  end
  
  case $current_probability
  when 0.003921569000000000..0.287225000000000000
    puts "I'm confident it is not [ #{$current_information} ]."
  when 0.287225000000000001..0.522225000000000000
    puts "I'm less unconfident it is not [ #{$current_information} ]."
  when 0.522225000000000001..0.756112500000000000
    puts "I'm almost sure it is [ #{$current_information} ]."
  when 0.756112500000000001..0.999999999999999999
    puts "I'm sure it is [ #{$current_information} ]."

  else
    puts "The probability is either to low or to large, so I can't determine exactly."
  end
  
  $current_probability = $current_probability + $current_probability
end

def reconsider
  if $current_probability > 0.999999999999999999
    $current_probability = 0.9 / $current_probability
  end

  case $current_probability
  when 0.003921569000000000..0.287225000000000000
    puts "I'm confident it is not [ #{$current_information} ]."
  when 0.287225000000000001..0.522225000000000000
    puts "I'm less unconfident it is not [ #{$current_information} ]."
  when 0.522225000000000001..0.756112500000000000
    puts "I'm almost sure it is [ #{$current_information} ]."
  when 0.756112500000000001..0.999999999999999999
    puts "I'm sure it is [ #{$current_information} ]."
  else
    puts "The probability is either to low or to large, so I can't determine exactly."
  end
  
  $current_probability = $current_probability * $current_probability
end

def dynamic_reward_allocation
  l1_reasses = "level one reasses"
  l2_reasses = "level two reasses"
  l3_reasses = "level tre reasses"
  l4_reasses = "level fro reasses"

  reward_model = [
    [[l1_reasses, l1_reasses, l1_reasses, l1_reasses],
     [l1_reasses, l1_reasses, l1_reasses, l2_reasses],
     [l1_reasses, l1_reasses, l1_reasses, l3_reasses],
     [l1_reasses, l1_reasses, l1_reasses, l4_reasses]],
   
    [[l2_reasses, l2_reasses, l2_reasses, l1_reasses],
     [l2_reasses, l2_reasses, l2_reasses, l2_reasses],
     [l2_reasses, l2_reasses, l2_reasses, l3_reasses],
     [l2_reasses, l2_reasses, l2_reasses, l4_reasses]],
   
    [[l3_reasses, l3_reasses, l3_reasses, l1_reasses],
     [l3_reasses, l3_reasses, l3_reasses, l2_reasses],
     [l3_reasses, l3_reasses, l3_reasses, l3_reasses],
     [l3_reasses, l3_reasses, l3_reasses, l4_reasses]],
   
    [[l4_reasses, l4_reasses, l4_reasses, l1_reasses],
     [l4_reasses, l4_reasses, l4_reasses, l2_reasses],
     [l4_reasses, l4_reasses, l4_reasses, l3_reasses],
     [l4_reasses, l4_reasses, l4_reasses, l4_reasses]],
  ]

  row_options = [0, 1, 2, 3]
  col_options = [0, 1, 2, 3]
  arr_options = [0, 1, 2, 3]

  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample

  current_reward_structure = reward_model[cur_row][cur_col][cur_arr]

  if    current_reward_structure == l1_reasses; reasses
  elsif current_reward_structure == l2_reasses; 2.times do reasses end
  elsif current_reward_structure == l3_reasses; 3.times do reasses end
  elsif current_reward_structure == l4_reasses; 4.times do reasses end
  else
    reconsider
  end
end

###########################################################################################################################
#                                            Get Larger Control Group Size                                                #
###########################################################################################################################
def get_larger_control_group(a1, a2, b1, b2,
                             c1, c2, d1, d2,
                             e1, e2, f1, f2,
                             g1, g2, h1, h2)

  a = "[ #{a1} ] [ #{a2} ]"
  b = "[ #{b1} ] [ #{b2} ]"
  c = "[ #{c1} ] [ #{c2} ]"
  d = "[ #{d1} ] [ #{d2} ]"
  e = "[ #{e1} ] [ #{e2} ]"
  f = "[ #{f1} ] [ #{f2} ]"
  g = "[ #{g1} ] [ #{g2} ]"
  h = "[ #{h1} ] [ #{h2} ]"
  
  labels = [
    [[a, a, a, a, a, a, a, a],
     [a, a, a, a, a, a, a, b],
     [a, a, a, a, a, a, a, c],
     [a, a, a, a, a, a, a, d],
     [a, a, a, a, a, a, a, e],
     [a, a, a, a, a, a, a, f],
     [a, a, a, a, a, a, a, g],
     [a, a, a, a, a, a, a, h]],
  
    [[b, b, b, b, b, b, b, a],
     [b, b, b, b, b, b, b, b],
     [b, b, b, b, b, b, b, c],
     [b, b, b, b, b, b, b, d],
     [b, b, b, b, b, b, b, e],
     [b, b, b, b, b, b, b, f],
     [b, b, b, b, b, b, b, g],
     [b, b, b, b, b, b, b, h]],

    [[c, c, c, c, c, c, c, a],
     [c, c, c, c, c, c, c, b],
     [c, c, c, c, c, c, c, c],
     [c, c, c, c, c, c, c, d],
     [c, c, c, c, c, c, c, e],
     [c, c, c, c, c, c, c, f],
     [c, c, c, c, c, c, c, g],
     [c, c, c, c, c, c, c, h]],
  
    [[d, d, d, d, d, d, d, a],
     [d, d, d, d, d, d, d, b],
     [d, d, d, d, d, d, d, c],
     [d, d, d, d, d, d, d, d],
     [d, d, d, d, d, d, d, e],
     [d, d, d, d, d, d, d, f],
     [d, d, d, d, d, d, d, g],
     [d, d, d, d, d, d, d, h]],
  
    [[e, e, e, e, e, e, e, a],
     [e, e, e, e, e, e, e, b],
     [e, e, e, e, e, e, e, c],
     [e, e, e, e, e, e, e, d],
     [e, e, e, e, e, e, e, e],
     [e, e, e, e, e, e, e, f],
     [e, e, e, e, e, e, e, g],
     [e, e, e, e, e, e, e, h]],
  
    [[f, f, f, f, f, f, f, a],
     [f, f, f, f, f, f, f, b],
     [f, f, f, f, f, f, f, c],
     [f, f, f, f, f, f, f, d],
     [f, f, f, f, f, f, f, e],
     [f, f, f, f, f, f, f, f],
     [f, f, f, f, f, f, f, g],
     [f, f, f, f, f, f, f, h]],
  
    [[g, g, g, g, g, g, g, a],
     [g, g, g, g, g, g, g, b],
     [g, g, g, g, g, g, g, c],
     [g, g, g, g, g, g, g, d],
     [g, g, g, g, g, g, g, e],
     [g, g, g, g, g, g, g, f],
     [g, g, g, g, g, g, g, g],
     [g, g, g, g, g, g, g, h]],
  
    [[h, h, h, h, h, h, h, a],
     [h, h, h, h, h, h, h, b],
     [h, h, h, h, h, h, h, c],
     [h, h, h, h, h, h, h, d],
     [h, h, h, h, h, h, h, e],
     [h, h, h, h, h, h, h, f],
     [h, h, h, h, h, h, h, g],
     [h, h, h, h, h, h, h, h]],
  ]
  
  statistics = [
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
   
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
   
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
   
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
   
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
   
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
   
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
     
    [[0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111], # H
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111],
     [0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.170888235, 0.111111111]],
  ]
  
  row_options = [0, 1, 2, 3, 4, 5, 6, 7]
  col_options = [0, 1, 2, 3, 4, 5, 6, 7]
  arr_options = [0, 1, 2, 3, 4, 5, 6, 7]
  
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample
  
  row_probability       = 0.111111111
  col_probability       = 0.111111111
  selection_probability = row_probability * col_probability
  
  labels_and_descriptions                = labels[cur_row][cur_col][cur_arr]
  labels_and_descriptions_probabilitiies = statistics[cur_row][cur_col][cur_arr]
  
  $current_label_desc        = labels_and_descriptions
  $current_label_probability = selection_probability * labels_and_descriptions_probabilitiies
  
  $current_information = [ $current_label_desc, $current_label_probability ]
end

def reasses_larger_control_group
  if $current_label_probability > 0.999999999999999999
    $current_label_probability = 0.9 / $current_label_probability
  end
  
  case $current_label_probability
  when 0.0021097312920768333..0.287225000000000000
    puts "I'm confident it is not [ #{$current_information} ]."
    
    $current_label_probability = $current_label_probability + $current_label_probability
  when 0.287225000000000001..0.522225000000000000
    puts "I'm less unconfident it is not [ #{$current_information} ]."
    
    #$current_label_probability = $current_label_probability + $current_label_probability
  when 0.522225000000000001..0.756112500000000000
    puts "I'm almost sure it is [ #{$current_information} ]."
    
    #$current_label_probability = $current_label_probability + $current_label_probability
  when 0.756112500000000001..0.999999999999999999
    puts "I'm sure it is [ #{$current_information} ]."

    #$current_label_probability = $current_label_probability + $current_label_probability
  else
    #puts "The probability is either to low or to large, so I can't determine exactly."

    $current_label_probability = 0.9 / $current_label_probability
  end

  $current_label_probability = $current_label_probability + $current_label_probability
  $current_information = [ $current_label_desc, $current_label_probability ]
end

def reconsider_larger_control_group
  if $current_label_probability > 0.999999999999999999
    $current_label_probability = 0.9 / $current_label_probability
  end

  case $current_label_probability
  when 0.003921569000000000..0.287225000000000000
    puts "I'm confident it is not [ #{$current_information} ]."
  when 0.287225000000000001..0.522225000000000000
    puts "I'm less unconfident it is not [ #{$current_information} ]."
  when 0.522225000000000001..0.756112500000000000
    puts "I'm almost sure it is [ #{$current_information} ]."
  when 0.756112500000000001..0.999999999999999999
    puts "I'm sure it is [ #{$current_information} ]."
  else
    #puts "The probability is either to low or to large, so I can't determine exactly."
    
    $current_label_probability = 0.9 / $current_label_probability
  end
  
  $current_label_probability = $current_label_probability * $current_label_probability
  $current_information = [ $current_label_desc, $current_label_probability ]
end

###########################################################################################################################
#                                                 Platform Analytics                                                      #
###########################################################################################################################
def evaluate_malicious_users(e, f, g, h)
  user_percents = [ e, f, g ], [ :version0p1p0, :version0p1p1, :version_all ]

  github_clones = h

  human_users = user_percents[0][0], user_percents[1][1]
  bot_users   = user_percents[0][1], user_percents[1][0]
  malicious   = user_percents[0][2], user_percents[1][2]

  a = human_users
  b = bot_users
  c = malicious

  malicious_humans = human_users[0] * malicious[0], human_users[1]
  malicious_bots   = bot_users[0]   * malicious[0], bot_users[1]
  
  d = malicious_humans
  e = malicious_bots

  puts "[ #{malicious_humans[0]}, #{malicious_humans[1]} ] About #{malicious_humans[0] * 100} of malicious users are humans."
  puts "[ #{malicious_bots[0]}, #{malicious_bots[1]} ] About #{malicious_bots[0] * 100} of malicious users are bots."

  user_types = [
    [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]], [a[0], d[0]], [a[0], e[0]]],
    [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]], [b[0], d[0]], [b[0], e[0]]],
    [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]], [c[0], d[0]], [c[0], e[0]]],
    [[d[0], a[0]], [d[0], b[0]], [d[0], c[0]], [d[0], d[0]], [d[0], e[0]]],
    [[e[0], a[0]], [e[0], b[0]], [e[0], c[0]], [e[0], d[0]], [e[0], e[0]]],
  ], [
    [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]], [a[1], d[1]], [a[1], e[1]]],
    [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]], [b[1], d[1]], [b[1], e[1]]],
    [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]], [c[1], d[1]], [c[1], e[1]]],
    [[d[1], a[1]], [d[1], b[1]], [d[1], c[1]], [d[1], d[1]], [d[1], e[1]]],
    [[e[1], a[1]], [e[1], b[1]], [e[1], c[1]], [e[1], d[1]], [e[1], e[1]]],
  ]

  row_options = [0, 1, 2, 3, 4]
  col_options = [0, 1, 2, 3, 4]
  arr_options = [0, 1]

  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample

  current_version     = user_types[1][cur_row][cur_col][cur_arr]
  current_probability = user_types[0][cur_row][cur_col][cur_arr]

  puts "[ #{current_version}, #{current_probability} ]"

  malicious_github_humans = malicious_humans[0] / github_clones, user_percents[1][1]
  malicious_github_bots   = malicious_bots[0] / github_clones,   user_percents[1][0]

  print "[ #{malicious_github_humans[0]}, #{malicious_github_humans[1]} ] About #{malicious_github_humans[0] * 100} malicious github users are humans."; puts " "
  print "[ #{malicious_github_bots[0]}, #{malicious_github_bots[1]} ] About #{malicious_github_bots[0] * 100} malicious github users are bots."; puts " "
  
  $malicious_humans = malicious_humans[0]
  $malicious_bots   = malicious_bots[0]
end

def evaluate_github_visitors(a, b, c)
  get_revealed_traffic    = a / b.to_f
  get_cloaked_traffic     = 1 - get_revealed_traffic.to_f
  
  get_revealed_percentage = c

  get_revealed_malicious_ratio = get_revealed_traffic * get_revealed_percentage
  get_cloaked_malicious_ratio  = get_cloaked_traffic * get_revealed_percentage
    
  visitor_traffic = [:cloaked_traffic, :revealed_traffic, :percentrage_revealed_malicious], [get_revealed_traffic, get_cloaked_traffic, get_revealed_percentage]
  
  cloaked_probability  = [ visitor_traffic[0][0], visitor_traffic[1][0] ]
  revealed_probability = [ visitor_traffic[0][0], visitor_traffic[1][0] ]
  
  cloaked_malicious_users  = "The amount of cloaked malicious users are #{get_cloaked_malicious_ratio}."
  revealed_malicious_users = "The amount of revealed malicious users are #{get_revealed_malicious_ratio}."
  
  puts cloaked_malicious_users
  puts revealed_malicious_users
  
  @cloaked_probability  = cloaked_probability
  @revealed_probability = revealed_probability
  
  @cloaked_malicious_users  = cloaked_malicious_users
  @revealed_malicious_users = revealed_malicious_users
end

def evaluate_yt_stats(a, b, c, d, e, f, specified_percent)
  user_channels = [:main_channel, :secondary_channel, :tertiary_channel], [a, c, e]
  
  subscriber_count = [b, d, f]
  
  mc_human_users     = user_channels[0][0], subscriber_count[0]
  sc_human_users     = user_channels[0][1], subscriber_count[1]
  tc_human_users     = user_channels[0][2], subscriber_count[2]

  husers_mc = 1 - ( 1 / ( 166 * 0.55 ) )
  husers_sc = 1 - ( 1 / ( 87 * 0.55 ) )
  husers_tc = 1 - ( 1 / ( 17 * 0.55 ) )
  
  hbot_mc = 1 - husers_mc
  hbot_sc = 1 - husers_sc
  hbot_tc = 1 - husers_tc
 
  hmal_mc = hbot_mc * hbot_mc
  hmal_sc = hbot_sc * hbot_sc
  hmal_tc = hbot_tc * hbot_tc

  puts hmal_mc.class
  
  malicious_users_mc = 0.9 * ( specified_percent*mc_human_users[1] )
  malicious_users_sc = 0.9 * ( specified_percent*sc_human_users[1] )
  malicious_users_tc = 0.9 * ( specified_percent*tc_human_users[1] )
  
  puts "[ #{mc_human_users[0]} #{husers_mc}, #{hbot_mc} #{hmal_mc} ] The amount of malicious users is #{malicious_users_mc}"
  puts "[ #{sc_human_users[0]} #{husers_sc}, #{hbot_sc} #{hmal_sc} ] The amount of malicious users is #{malicious_users_sc}"
  puts "[ #{tc_human_users[0]} #{husers_tc}, #{hbot_tc} #{hmal_tc} ] The amount of malicious users is #{malicious_users_tc}"
end

def difference_in_traffic(a, b)
  differential_percent = a.to_f / b.to_f
  
  puts "The difference between Rubygems #{a} and Youtube #{b} is #{differential_percent} percent."
  
  $traffic_difference = "The difference between Rubygems #{a} and Youtube #{b} is #{differential_percent} percent."
end

###########################################################################################################################
#                                     Analyze Statistics For Known Vulnerabilities                                        #
###########################################################################################################################
def direct_translate(a)
  translate = {
    "dog"              =>           "U cine",
    "cat"              =>           "U cato",
    "mouse"            =>         "U souido",
    "werewolf"         =>         "U lugado",
    "bearman"          =>     "U oudofotoko", #oursotoko
    "legacy-vandalism" => "beqefutecenedade",
    "embrace"          =>         "etreinte",
    "ledge"            =>          "iwatana",
    "Gazing Psychic"   =>   "Gyoshiregardos",
    "gaze"             =>           "regard",
    "psychic"          =>         "reinosha",

    "U cine"           =>          "The dog",
    "U cato"           =>          "The cat",
    "U souido"         =>        "The mouse",
    "U lugado"         =>     "The werewolf",
    "U oudofotoko"     =>      "The bearman",
    "legacy-vandalism" => "beqefutecenedade",
    "etreinte"         =>          "embrace",
    "iwatana"          =>            "ledge",
    "Gyoshiregardos"   =>   "Gazing Psychic",
    "regard"           =>             "gaze",
    "psychic"          =>         "reinosha",

    "black"  =>  "ejori",
    "red"    => "douega",
    "green"  => "verute",
    "blue"   =>  "berue",
    "ejori"  =>  "black",
    "douega" =>    "red",
    "verute" =>  "green",
    "berue"  =>   "blue",
  }

  word_into_chunks = a.split(" ")
  size_limit       = word_into_chunks.size.to_i
  index = 0

  size_limit.times do
    current_word = word_into_chunks[index]

    print current_word
    print " "

    index = index + 1
  end
end

def adapt_words(a)
  adapt = {
    "A" => "A",
    "E" => "E",
    "I" => "I",
    "O" => "O",
    "U" => "U",
    "Y" => "Y",
    "B" => "B"
    "C" => "C"
    "D" => "D"
    "F" => "F"
    "G" => "G"
    "H" => "H"
    "J" => "J"
    "K" => "K"
    "L" => "H"
    "M" => "W"
    "N" => "J"
    "P" => "P"
    "Q" => "Q"
    "R" => "D"
    "S" => "S"
    "T" => "T"
    "V" => "V"
    "W" => "W"
    "X" => "X"
    "Z" => "Y"
  }
  
  word_into_chunks = a.split(" ")
  size_limit       = word_into_chunks.size.to_i
  index = 0

  size_limit.times do
    current_word = word_into_chunks[index]

    print adapt[current_word]
    print " "

    index = index + 1
  end
end

def analyze_statistics
  require "SelfModifiedDecisionTree"
      
  behaviours = RevisedBayes.new(:zero_vagueness,       :some_vagueness,       :mild_vagueness,       :medium_vagueness,             :high_vagueness),
               RevisedBayes.new(:zero_analytic_leaks,  :some_analytic_leaks,  :mild_analytic_leaks,  :medium_analytic_leaks,   :high_analytic_leaks),
               RevisedBayes.new(:zero_misuse_cloaking, :some_misuse_cloaking, :mild_misuse_cloaking, :medium_misuse_cloaking, :high_misuse_cloaking)
                   
  label_vagueness  = behaviours[0]
  analytic_leakage = behaviours[1]
  misuse_cloaking  = behaviours[2]
      
  ## Label Vagueness
  label_vagueness.train(:zero_vagueness,   ":starving_prisoners",           "zero vagueness")
  label_vagueness.train(:some_vagueness,   ":competitive_participants",     "some vagueness")
  label_vagueness.train(:mild_vagueness,   ":competitive_economics",        "mild vagueness")
  label_vagueness.train(:medium_vagueness, ":reverse_social_democracy",   "medium vagueness")
  label_vagueness.train(:high_vagueness,   ":studying_reverse_economics",   "high vagueness")
      
  ## Analytic Leakage
  analytic_leakage.train(:zero_analytic_leaks,   ":rubygems",                        "zero vagueness")
  analytic_leakage.train(:some_analytic_leaks,   ":software_dependencies",           "some vagueness")
  analytic_leakage.train(:mild_analytic_leaks,   ":imported_software_supplements",   "mild vagueness")
  analytic_leakage.train(:medium_analytic_leaks, ":supply_chain_necessities",      "medium vagueness")
  analytic_leakage.train(:high_analytic_leaks,   ":necessities_from_supplier",       "high vagueness")
      
  ## Misuse Cloaking
  misuse_cloaking.train(:zero_misuse_cloaking,   ":rubygems_analytics,                               'is used to analyze traffic on Rubygems.'",   "zero misuse cloaking")
  misuse_cloaking.train(:some_misuse_cloaking,   ":rubygems_analytics,                                 'is used to observe other web traffic.'",   "some misuse cloaking")
  misuse_cloaking.train(:mild_misuse_cloaking,   ":rubygems_analytics,                    'can be used to analyze different websites traffic.'",   "mild misuse cloaking")
  misuse_cloaking.train(:medium_misuse_cloaking, ":rubygems_analytics,   'can be used to analyze malicious traffic in LANs they dont control.'", "medium misuse cloaking")
  misuse_cloaking.train(:high_misuse_cloaking,   ":rubygems_analytics, 'there is no local area networks I can't jailbreak to observe traffic.'",   "high misuse cloaking")
      
  ## Classify Vagueness, Leakage, and Cloaking
      
  documents  = File.readlines("analyze.rb")
  size_limit = documents.size.to_i
  index      = 0
      
  size_limit.times do
    current_vagueness = label_vagueness.classify(documents[index])
    current_analytic  = analytic_leakage.classify(documents[index])
    current_cloaking  = misuse_cloaking.classify(documents[index])
        
    get_statistics(:label_vagueness,  "such as #{current_vagueness} is when a label and description is excessively vague as to not be meaningful to the uninitiated.",
                   :analytic_leakage,                "such as #{current_analytic} is when a label and description might leak analytic details to alert tresspassers.",
                   :misuse_cloaking,           "such as #{current_cloaking} is when a label and description is attempting to discuise its behaviours to seem benign.")
   
    4.times do reasses end
     
    index = index + 1
  end
end

def word_types
  require "SelfModifiedDecisionTree"
  
  word_origin = RevisedBayes.new(:french,           :not_french),
                RevisedBayes.new(:japanese,       :not_japanese),
                RevisedBayes.new(:german,           :not_german),
                RevisedBayes.new(:hybrid,           :not_hybrid),
                RevisedBayes.new(:unknown_origin, :known_origin)
                
  french_type   = word_origin[0]
  japanese_type = word_origin[1]
  german_type   = word_origin[2]
  hybrid_type   = word_origin[3]
  unknown_type  = word_origin[4]
  
  ## French Words
  french_type.train(:french,           "vous",           "vous")
  french_type.train(:french,             "je",             "je")
  french_type.train(:french,             "tu",             "tu")
  french_type.train(:french, "sabots du bois", "sabots du bois")
  
  french_type.train(:not_french,  "anata",  "anata")
  french_type.train(:not_french,     "du",     "du")
  french_type.train(:not_french, "atashi", "atashi")
  french_type.train(:not_french,    "Ich",    "ich")
  
  ## Japanese Words
  japanese_type.train(:japanese,               "konnichiwa",               "konnichiwa")
  japanese_type.train(:japanese,                  "kudasai",                  "kudasai")
  japanese_type.train(:japanese, "konnichiwa anata kudasai", "konnichiwa anata kudasai")
  
  japanese_type.train(:not_japanese,               "bonjour",               "bonjour")
  japanese_type.train(:not_japanese,            "gutemorgen",            "gutemorgen")
  japanese_type.train(:not_japanese, "Je mange des spatzle.", "je mange des spatzle.")
  
  ## German Words
  german_type.train(:german,         "du",         "du")
  german_type.train(:german,        "ich",        "ich")
  german_type.train(:german, "gutemorgen", "gutemorgen")
  
  german_type.train(:not_german,     "de",     "de")
  german_type.train(:not_german, "atashi", "atashi")
  german_type.train(:not_german,  "anata",  "anata")
  
  ## Hybrid Words
  hybrid_type.train(:hybrid,       "revesukyana",   "revesukyana")
  hybrid_type.train(:hybrid,     "oiseausykyana", "oiseausukyana")
  hybrid_type.train(:hybrid,     "omuleisetango", "omuleisetango")
  
  hybrid_type.train(:not_hybrid,          "revemoire",          "revemoire")
  hybrid_type.train(:not_hybrid, "Bequest De Cendres", "Bequest De Cendres")
  
  ## Unknown Origin
  unknown_type.train(:unknown_origin,      "tiwadano",      "tiwadano")
  unknown_type.train(:known_origin,   "omuleisetango", "omuleisetango")
  
  ### Classification
  document   = File.readlines("documents/text.txt")
  size_limit = document.size.to_i
  index      = 0
  
  size_limit.times do
    french_classification   = french_type.classify(document[index])
    japanese_classification = japanese_type.classify(document[index])
    german_classification   = german_type.classify(document[index])
    hybrid_classification   = hybrid_type.classify(document[index])
    unknown_classification  = unknown_type.classify(document[index])
    
    ## French data
    french_label = french_classification[0]
    french_probability = french_classification[1]
    
    ## Japanese data
    japanese_label       = japanese_classification[0]
    japanese_probability = japanese_classification[1]
    
    ## German data
    german_label       = german_classification[0]
    german_probability = german_classification[1]
        
    ## Hybrid data
    hybrid_label       = hybrid_classification[0]
    hybrid_probability = hybrid_classification[1]
     
    ## Unknown data
    unknown_label       = french_classification[0]
    unknown_probability = french_classification[1]
    
    ### Get Statistics
    get_statistics(:french_origin,       "This phrase appears to be of #{french_label} for #{french_probability}.",
                   :japanese_origin, "This phrase appears to be of #{japanese_label} for #{japanese_probability}.",
                   :german_origin,       "This phrase appears to be of #{german_label} for #{german_probability}.")
                   
    dynamic_reward_allocation

    get_statistics(:hybrid_origin,         "This phrase appears to be of #{hybrid_label} for #{hybrid_probability}.",
                   :unknown_origin,      "This phrase appears to be of #{unknown_label} for #{unknown_probability}.",
                   :inconclusive_origin, "I need more information about the origin as the results are inconclusive.")

    dynamic_reward_allocation
  end
end

def word_types2(a, c, d, e, f)
  require "SelfModifiedDecisionTree"
  
  word_origin = RevisedBayes.new(:french,           :not_french),
                RevisedBayes.new(:japanese,       :not_japanese),
                RevisedBayes.new(:german,           :not_german),
                RevisedBayes.new(:hybrid,           :not_hybrid),
                RevisedBayes.new(:unknown_origin, :known_origin)
                
  french_type   = word_origin[0]
  japanese_type = word_origin[1]
  german_type   = word_origin[2]
  hybrid_type   = word_origin[3]
  unknown_type  = word_origin[4]
  
  ## French Words
  french_type.train(:french,           "vous",           "vous")
  french_type.train(:french,             "je",             "je")
  french_type.train(:french,             "tu",             "tu")
  french_type.train(:french, "sabots du bois", "sabots du bois")
  
  french_type.train(:not_french,  "anata",  "anata")
  french_type.train(:not_french,     "du",     "du")
  french_type.train(:not_french, "atashi", "atashi")
  french_type.train(:not_french,    "Ich",    "ich")
  
  ## Japanese Words
  japanese_type.train(:japanese,               "konnichiwa",               "konnichiwa")
  japanese_type.train(:japan+ese,                  "kudasai",                  "kudasai")
  japanese_type.train(:japanese, "konnichiwa anata kudasai", "konnichiwa anata kudasai")
  
  japanese_type.train(:not_japanese,               "bonjour",               "bonjour")
  japanese_type.train(:not_japanese,            "gutemorgen",            "gutemorgen")
  japanese_type.train(:not_japanese, "Je mange des spatzle.", "je mange des spatzle.")
  
  ## German Words
  german_type.train(:german,         "du",         "du")
  german_type.train(:german,        "ich",        "ich")
  german_type.train(:german, "gutemorgen", "gutemorgen")
  
  german_type.train(:not_german,     "de",     "de")
  german_type.train(:not_german, "atashi", "atashi")
  german_type.train(:not_german,  "anata",  "anata")
  
  ## Hybrid Words
  hybrid_type.train(:hybrid,       "revesukyana",   "revesukyana")
  hybrid_type.train(:hybrid,     "oiseausykyana", "oiseausukyana")
  hybrid_type.train(:hybrid,     "omuleisetango", "omuleisetango")
  
  hybrid_type.train(:not_hybrid,          "revemoire",          "revemoire")
  hybrid_type.train(:not_hybrid, "Bequest De Cendres", "Bequest De Cendres")
  
  ## Unknown Origin
  unknown_type.train(:unknown_origin,      "tiwadano",      "tiwadano")
  unknown_type.train(:known_origin,   "omuleisetango", "omuleisetango")
  
  ### Classification
  #document   = File.readlines("documents/text.txt")
  #size_limit = documents.size.to_i
  #main_index      = 0
  
  french_classification   = french_type.classify(a)
  japanese_classification = japanese_type.classify(b)
  german_classification   = german_type.classify(c)
  hybrid_classification   = hybrid_type.classify(d)
  unknown_classification  = unknown_type.classify(e)
    
  ## French data
  french_label = french_classification[0]
  french_probability = french_classification[0]
    
  ## Japanese data
  japanese_label       = japanese_classification[0]
  japanese_probability = japanese_classification[0]
    
  ## German data
  german_label       = german_classification[0]
  german_probability = german_classification[0]
        
  ## Hybrid data
  hybrid_label       = hybrid_classification[0]
  hybrid_probability = hybrid_classification[0]
     
  ## Unknown data
  unknown_label = french_classification[0]
  unknown_probability = french_classification[0]
    
  ### Get Statistics
  get_statistics(:french_origin,       "This phrase appears to be of #{french_label} for #{french_probability}.",
                 :japanese_origin, "This phrase appears to be of #{japanese_label} for #{japanese_probability}.",
                 :german_origin,       "This phrase appears to be of #{german_label} for #{german_probability}.")
                   
  get_statistics(:hybrid_origin,         "This phrase appears to be of #{hybrid_label} for #{hybrid_probability}.",
                 :unknown_origin,      "This phrase appears to be of #{unknown_label} for #{unknown_probability}.",
                 :inconclusive_origin, "I need more information about the origin as the results are inconclusive.")
end

def legacy_ahuzacos
  require "SelfModifiedDecisionTree"

  a = RevisedBayes.new(
                     # Alphabetic components
                     :vowel,
                     :semivowel,
                     :consonant,
                     :doubleconsonant,
                     :dipthong,
                     :trithong,

                     # Grammatical components
                     :word_gender,             :noun,
                     :adjective,        :conjunction,
                     :preposition,             :verb,
                     :adverb,           :punctuation,
                     :personal_pronoun, :honorificos,

                     # Sentence formats
                     :question, :answer, :request,
  )

  # Single character training
  # These functions as part of the standard Latin alphabet minus some characters.

  ## Vowels
  a.train(:vowel, "a", "word"); a.train(:vowel, "e", "word");
  a.train(:vowel, "i", "word"); a.train(:vowel, "o", "word");
  a.train(:vowel, "u", "word")

  # Semivowels
  a.train(:semivowel, "x", "word"); a.train(:semivowel, "y", "word")

  # Consonants
  a.train(:consonant, "b", "word"); a.train(:consonant, "c", "word");
  a.train(:consonant, "g", "word"); a.train(:consonant, "k", "word");
  a.train(:consonant, "m", "word"); a.train(:consonant, "p", "word");
  a.train(:consonant, "q", "word"); a.train(:consonant, "t", "word");
  a.train(:consonant, "u", "word"); a.train(:consonant, "v", "word");
  a.train(:consonant, "w", "word"); a.train(:consonant, "z", "word");

  # Double Consonants
  a.train(:doubleconsonant, "d", "word"); a.train(:doubleconsonant, "f", "word");
  a.train(:doubleconsonant, "j", "word"); a.train(:doubleconsonant, "n", "word");
  a.train(:doubleconsonant, "l", "word"); a.train(:doubleconsonant, "r", "word");
  a.train(:doubleconsonant, "s", "word"); a.train(:doubleconsonant, "l", "word");
  a.train(:doubleconsonant, "m", "word")

  # Dipthongs and Tripthongs
  # These function as the sounds for the Angel script font.

  # Section B
  a.train(:dipthong, "ba", "word"); a.train(:dipthong, "be", "word");
  a.train(:dipthong, "bi", "word"); a.train(:dipthong, "bo", "word");
  a.train(:dipthong, "bu", "word")

  # Section C
  a.train(:dipthong, "ca", "word"); a.train(:dipthong, "ce", "word");
  a.train(:dipthong, "ci", "word"); a.train(:dipthong, "co", "word");
  a.train(:dipthong, "cu", "word")

  # Section G
  a.train(:dipthong, "ga", "word"); a.train(:dipthong, "ge", "word");
  a.train(:dipthong, "gi", "word"); a.train(:dipthong, "go", "word");
  a.train(:dipthong, "gu", "word")

  # Section K
  a.train(:dipthong, "ka", "word"); a.train(:dipthong, "ke", "word");
  a.train(:dipthong, "ki", "word"); a.train(:dipthong, "ko", "word");
  a.train(:dipthong, "ku", "word")

  # Section M
  a.train(:dipthong, "ma", "word"); a.train(:dipthong, "me", "word");
  a.train(:dipthong, "mi", "word"); a.train(:dipthong, "mo", "word");
  a.train(:dipthong, "mu", "word")

  # Section P
  a.train(:dipthong, "pa", "word"); a.train(:dipthong, "pe", "word");
  a.train(:dipthong, "pi", "word"); a.train(:dipthong, "po", "word");
  a.train(:dipthong, "pu", "word")

  # Section Q
  a.train(:dipthong, "qa", "word"); a.train(:dipthong, "qe", "word");
  a.train(:dipthong, "qi", "word"); a.train(:dipthong, "qo", "word");
  a.train(:dipthong, "qu", "word")

  # Section T
  a.train(:dipthong, "ta", "word"); a.train(:dipthong, "te", "word");
  a.train(:dipthong, "ti", "word"); a.train(:dipthong, "to", "word");
  a.train(:dipthong, "tu", "word")

  # Section V
  a.train(:dipthong, "va", "word"); a.train(:dipthong, "ve", "word");
  a.train(:dipthong, "vi", "word"); a.train(:dipthong, "vo", "word");
  a.train(:dipthong, "vu", "word")

  # Section Z
  a.train(:dipthong, "za", "word"); a.train(:dipthong, "ze", "word");
  a.train(:dipthong, "zi", "word"); a.train(:dipthong, "zo", "word");
  a.train(:dipthong, "zu", "word")

  # Phonetically tripthongs
  ## Section D
  a.train(:trithong, "da", "word"); a.train(:trithong, "de", "word");
  a.train(:trithong, "di", "word"); a.train(:trithong, "do", "word");
  a.train(:trithong, "du", "word")

  ## Section S
  a.train(:trithong, "sa", "word"); a.train(:trithong, "se", "word");
  a.train(:trithong, "si", "word"); a.train(:trithong, "so", "word");
  a.train(:trithong, "su", "word")

  ## Section H
  a.train(:trithong, "ha", "word"); a.train(:trithong, "he", "word");
  a.train(:trithong, "hi", "word"); a.train(:trithong, "ho", "word");
  a.train(:trithong, "hu", "word")

  ## Section N
  a.train(:trithong, "ba", "word"); a.train(:trithong, "be", "word");
  a.train(:trithong, "bi", "word"); a.train(:trithong, "bo", "word");
  a.train(:trithong, "bu", "word")

  ## Sections X
  a.train(:trithong, "xa", "word"); a.train(:trithong, "xe", "word");
  a.train(:trithong, "xi", "word"); a.train(:trithong, "xo", "word");
  a.train(:trithong, "xu", "word")

  # Specific constructed language words.
  ## Honorificos
  #a.train(:honorificos, "Goz", "word")

  ## Word Genders

  ## Nouns
  a.train(:noun,             "ahusacos", "word"); a.train(:noun,    "baozeng", "word");
  a.train(:noun, "koneverogenitodevute", "word"); a.train(:noun,   "coranore", "word");
  a.train(:noun,             "corasuhe", "word"); a.train(:noun,     "domche", "word");
  a.train(:noun,         "resetudanite", "word"); a.train(:noun,      "zemma", "word");
  a.train(:noun,                 "zhhe", "word"); a.train(:noun,     "gadcon", "word");
  a.train(:noun,            "gapacadde", "word"); a.train(:noun,      "homme", "word");
  a.train(:noun,            "ipohetiku", "word"); a.train(:noun,       "mede", "word");
  a.train(:noun,                "amase", "word"); a.train(:noun, "nunonorede", "word");
  a.train(:noun,           "nunedesude", "word"); a.train(:noun,      "onche", "word");
  a.train(:noun,          "oraretareve", "noun"); a.train(:noun,  "urusegaru", "word");
  a.train(:noun,                 "pede", "noun"); a.train(:noun, "zamurapohe", "word");
  a.train(:noun,                "zoeud", "noun"); a.train(:noun,     "zommez", "word");
  a.train(:noun,                "tante", "noun"); a.train(:noun, "vidixadixu", "word");
  a.train(:noun,          "arbrejakata", "noun"); a.train(:noun,      "adede", "word");
  a.train(:noun,            "ibadereva", "noun");

  ## Personal Pronoun
  a.train(:personal_pronoun, "Ez", "word"); a.train(:personal_pronoun, "Ehhe", "word");

  ## Adjectives
  a.train(:adjective,        "bheu", "word"); a.train(:adjective,             "bdaun", "word");
  a.train(:adjective,       "naune", "word"); a.train(:adjective,         "nkkogomen", "word");
  a.train(:adjective,       "mauve", "word"); a.train(:adjective,               "nod", "word");
  a.train(:adjective,      "odange", "word"); a.train(:adjective,    "dadzumu-mnmahe", "word");
  a.train(:adjective,       "rouge", "word"); a.train(:adjective,             "zangu", "word");
  a.train(:adjective, "kendosavate", "word"); a.train(:adjective, "xosfexinipeneture", "word");

  ## Verb
  a.train(:verb,         "azfetere", "word"); a.train(:verb, "ibe", "word");
  a.train(:verb,          "vohette", "word");

  ## Adverb
  a.train(:adverb, "vedt", "word")

  ## Conjucation
  a.train(:conjunction, "cette", "word");

  ## Sections for testing individual characters.
  b = File.readlines("_input/alphabet_set.txt")

  interval = 0

  total_size = b.size.to_i

  total_size.times do
    read_line = b[interval].to_s.split(" ")

    f.print a.classify(read_line)
    f.puts " "

    interval = interval + 1
  end
end

################################################################################################
#                                     Maisette Functions                                       #
################################################################################################
def parle(phrase)
  puts phrase
end

def bokette(phrase)
  sleep(1.5)
  
  puts phrase
end

def cette(x, y)
  if x == ""
    puts "X is not provided."

    abort
  end
  
  if y == ""
    puts "X is not provided."

    abort
  end

  "Cette #{x} es #{y}. "
end

def bokecette(x, y)
  if x == ""
    puts "X is not provided."

    abort
  end
  
  if y == ""
    puts "X is not provided."

    abort
  end

  sleep(1.5)

  "Cette #{x} es #{y}. "
end

def maisette(x, y)
  if x == ""
    puts "X is not provided."

    abort
  end
  
  if y == ""
    puts "X is not provided."

    abort
  end

  "Mais cette #{x} es ne #{y}."
end

def bokemaisette(x, y)
  if x == ""
    puts "X is not provided."

    abort
  end
  
  if y == ""
    puts "X is not provided."

    abort
  end

  sleep(1.5)

  "Mais cette #{x} es ne #{y}."
end

def sinon(x, y, z)
  if x == ""
    puts "X is not provided."

    abort
  end
  
  if y == ""
    puts "X is not provided."

    abort
  end

  if z == ""
    puts "X is not provided."

    abort
  end

  "Sinon #{x} es ne #{y} ou #{z}."
end

def bokesinon(x, y, z)
  if x == ""
    puts "X is not provided."

    abort
  end
  
  if y == ""
    puts "X is not provided."

    abort
  end

  if z == ""
    puts "X is not provided."

    abort
  end

  sleep(1.5)

  "Sinon #{x} es ne #{y} ou #{z}."
end

def ouvert(a, b, c, d, e, g)
  File.open(a, "a") { |f|
    f.puts b
  }

  File.open(c, "a") { |f|
    f.puts d
  }

  File.open(e, "a") { |f|
    f.puts g
  }
end
  
## Moves the old file to an isolated directory. A is the old file name, B is the new directory. C is equal to the old file name, D is the placehodler txt.
def wabisab(a, b, c)
  # wabisab("hello.txt", "hello", "hello.txt", "placeholder text")
  
  old_file = File.read(a)
  
  new_file = "#{old_file}"
  
  system("rm #{a}; mkdir #{b}")
  
  File.open("#{b}/#{a}", "w") { |f|
    f.puts new_file
  }
  
  File.open("#{c}", "w") { |f|
    f.puts d
  }
end

def lingua_dice(a, b, c, d, e, f)
  matrix = [
    [[a, a], [a, b], [a, c], [a, d], [a, e], [a, f]],
    [[b, a], [b, b], [b, c], [b, d], [b, e], [b, f]],
    [[c, a], [c, b], [c, c], [c, d], [c, e], [c, f]],
    [[d, a], [d, b], [d, c], [d, d], [d, e], [d, f]],
    [[e, a], [e, b], [e, c], [e, d], [e, e], [e, f]],
    [[f, a], [f, b], [f, c], [f, d], [f, e], [f, f]],
  ], [
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
  ]
      
  row_probability = 0.166666667
  col_probability = 0.166666667
      
  selection_probability = row_probability * col_probability
      
  row_options = [0, 1, 2, 3, 4, 5]
  col_options = [0, 1, 2, 3, 4, 5]
  arr_options = [0, 1]
      
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample

  dice_roll = matrix[0][cur_row][cur_col][cur_arr], matrix[1][cur_row][cur_col][cur_arr]
      
  d1 = dice_roll[0]
  d2 = dice_roll[1]

  puts "[#{d1}, #{d2}], # #{selection_probability} For Row #{cur_row} And #{cur_col}"
            
  Maisettelingua::LinguaFranca.ouvert("lingua_dice/phrase_selection/dice_rolls.txt",                              "#{d1}",
                                      "lingua_dice/phrase_probability/dice_rolls.txt",                            "#{d2}",
                                      "lingua_dice/phrase_matrix_probability/dice_rolls.txt", "#{selection_probability}")
end

def lingua_advanced(a, b, c, d, e, f)
  matrix = [
    [[a, a], [a, b], [a, c], [a, d], [a, e], [a, f]],
    [[b, a], [b, b], [b, c], [b, d], [b, e], [b, f]],
    [[c, a], [c, b], [c, c], [c, d], [c, e], [c, f]],
    [[d, a], [d, b], [d, c], [d, d], [d, e], [d, f]],
    [[e, a], [e, b], [e, c], [e, d], [e, e], [e, f]],
    [[f, a], [f, b], [f, c], [f, d], [f, e], [f, f]],
  ], [
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
    [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
  ]
      
  row_probability = 0.166666667
  col_probability = 0.166666667
      
  selection_probability = row_probability * col_probability
      
  row_options = [0, 1, 2, 3, 4, 5]
  col_options = [0, 1, 2, 3, 4, 5]
  arr_options = [0, 1]
      
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample

  dice_roll = matrix[0][cur_row][cur_col][cur_arr], matrix[1][cur_row][cur_col][cur_arr]
      
  d1 = dice_roll[0]
  d2 = dice_roll[1]

  puts "[#{d1}, #{d2}], # #{selection_probability} For Row #{cur_row} And #{cur_col}"
            
  #Maisettelingua::LinguaFranca.ouvert("lingua_dice/phrase_selection/dice_rolls.txt",                              "#{d1}",
  #                                    "lingua_dice/phrase_probability/dice_rolls.txt",                            "#{d2}",
  #                                    "lingua_dice/phrase_matrix_probability/dice_rolls.txt", "#{selection_probability}")
                                      
  get_statistics(:phrase_selection,                             "#{d1}",
                 :phrase_probability,                           "#{d2}",
                 :phrase_matrix_probability, "#{selection_probability}")
                 
  reasses
  reasses
  reasses
end

################################################################################################
#                                Conveyor Belt Like Mechanics                                  #
################################################################################################
def create_rulesets
  row = 0 ## The value of row is 0.

  ## The directory knwon_ruleset is _ruleset/rules.txt
  ruleset = File.readlines("_rulesets/rules.txt")

  ### Values of the individual rows, as Ruby counts from zero.
  rule1 = 0 #row + 1
  rule2 = 1 #row + 2
  rule3 = 2 #row + 3
  rule4 = 3 #row + 4
  rule5 = 4 #row + 5
  rule6 = 5 #row + 6
  rule7 = 6 #row + 7
  rule8 = 7 #row + 8
  rule9 = 8 #row + 9

  first_rule    = ruleset[rule1].tr "
", ""
  second_rule   = ruleset[rule2].tr "
", ""
  third_rule    = ruleset[rule3].tr "
", ""
  fourth_rule   = ruleset[rule4].tr "
", ""
  fifth_rule    = ruleset[rule5].tr "
", ""
  sixth_rule    = ruleset[rule6].tr "
", ""
  seventh_rule  = ruleset[rule7].tr "
", ""
  eighth_rule   = ruleset[rule8].tr "
", ""
  nineth_rule   = ruleset[rule9].tr "
", ""

  ### Creation of the actual rulesets.
  ruleset_1 = "#{first_rule}
#{second_rule}
#{third_rule}
#{fourth_rule}
#{fifth_rule}
#{sixth_rule}
#{seventh_rule}
#{eighth_rule}
#{nineth_rule}"

  ruleset_2 = "#{nineth_rule}
#{first_rule}
#{second_rule}
#{third_rule}
#{fourth_rule}
#{fifth_rule}
#{sixth_rule}
#{seventh_rule}
#{eighth_rule}"

  ruleset_3 = "#{eighth_rule}
#{nineth_rule}
#{first_rule}
#{second_rule}
#{third_rule}
#{fourth_rule}
#{fifth_rule}
#{sixth_rule}
#{seventh_rule}"

  ruleset_4 = "#{seventh_rule}
#{eighth_rule}
#{nineth_rule}
#{first_rule}
#{second_rule}
#{third_rule}
#{fourth_rule}
#{fifth_rule}
#{sixth_rule}"

  ruleset_5 = "#{sixth_rule}
#{seventh_rule}
#{eighth_rule}
#{nineth_rule}
#{first_rule}
#{second_rule}
#{third_rule}
#{fourth_rule}
#{fifth_rule}"

  ruleset_6 = "#{fifth_rule}
#{sixth_rule}
#{seventh_rule}
#{eighth_rule}
#{nineth_rule}
#{first_rule}
#{second_rule}
#{third_rule}
#{fourth_rule}"

  ruleset_7 = "#{fourth_rule}
#{fifth_rule}
#{sixth_rule}
#{seventh_rule}
#{eighth_rule}
#{nineth_rule}
#{first_rule}
#{second_rule}
#{third_rule}"

  ruleset_8 = "#{third_rule}
#{fourth_rule}
#{fifth_rule}
#{sixth_rule}
#{seventh_rule}
#{eighth_rule}
#{nineth_rule}
#{first_rule}
#{second_rule}"

  ruleset_9 = "#{second_rule}
#{third_rule}
#{fourth_rule}
#{fifth_rule}
#{sixth_rule}
#{seventh_rule}
#{eighth_rule}
#{nineth_rule}
#{first_rule}"

  open("_adaptation/ruleset_shift1.txt", "w") { |f|
    f.puts ruleset_1
  }

  open("_adaptation/ruleset_shift2.txt", "w") { |f|
    f.puts ruleset_2
  }

  open("_adaptation/ruleset_shift3.txt", "w") { |f|
    f.puts ruleset_3
  }

  open("_adaptation/ruleset_shift4.txt", "w") { |f|
    f.puts ruleset_4
  }

  open("_adaptation/ruleset_shift5.txt", "w") { |f|
    f.puts ruleset_5
  }

  open("_adaptation/ruleset_shift6.txt", "w") { |f|
    f.puts ruleset_6
  }

  open("_adaptation/ruleset_shift7.txt", "w") { |f|
    f.puts ruleset_7
  }

  open("_adaptation/ruleset_shift8.txt", "w") { |f|
    f.puts ruleset_8
  }

  open("_adaptation/ruleset_shift9.txt", "w") { |f|
    f.puts ruleset_9
  }
end

def adaptation_vortex
  ## Create new sound_file with new ruleset permutation.
  ruleset_choice = File.read("_rulesets/choice/value.txt").to_s.to_i

  # Resets reset choice if at 8 in Ruby.
  if ruleset_choice > 8
    ruleset_choice = 0
  end

  mutation1 = File.readlines("_adaptation/ruleset_shift1.txt")
  mutation2 = File.readlines("_adaptation/ruleset_shift2.txt")
  mutation3 = File.readlines("_adaptation/ruleset_shift3.txt")
  mutation4 = File.readlines("_adaptation/ruleset_shift4.txt")
  mutation5 = File.readlines("_adaptation/ruleset_shift5.txt")
  mutation6 = File.readlines("_adaptation/ruleset_shift6.txt")
  mutation7 = File.readlines("_adaptation/ruleset_shift7.txt")
  mutation8 = File.readlines("_adaptation/ruleset_shift8.txt")
  mutation9 = File.readlines("_adaptation/ruleset_shift9.txt")

  ruleset_list = [
    mutation1, mutation2, mutation3,
    mutation4, mutation5, mutation6,
    mutation7, mutation8, mutation9,
  ]

  chosen_ruleset = ruleset_list[ruleset_choice]

  ## These are the rows.
  vortex_row_1 = 0
  vortex_row_2 = 1
  vortex_row_4 = 3
  vortex_row_8 = 7
  vortex_row_7 = 6
  vortex_row_5 = 4

  ## Performs standard subroutines.
  system("#{chosen_ruleset[vortex_row_1]}")
  system("#{chosen_ruleset[vortex_row_2]}")
  system("#{chosen_ruleset[vortex_row_4]}")
  system("#{chosen_ruleset[vortex_row_8]}")
  system("#{chosen_ruleset[vortex_row_7]}")
  system("#{chosen_ruleset[vortex_row_5]}")
  system("#{chosen_ruleset[vortex_row_1]}")

  open("_rulesets/choice/value.txt", "w") { |f|
    ruleset_choice = ruleset_choice + 1

    f.puts ruleset_choice
  }
end

################################################################################################
#                                Self Refining Search Engine                                   #
################################################################################################
def self.express_behaviours
  object = [
    [["key",         "key"], ["key",         "magic stone"], ["key",         "sword"]], 
    [["magic stone", "key"], ["magic stone", "magic stone"], ["magic stone", "sword"]],
    [["sword",       "key"], ["sword",       "magic stone"], ["sword",       "sword"]],
  ]
	  
  row_options = [0, 1, 2]
  col_options = [0, 1, 2]
  arr_options = [0, 1]
	  
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample
	  
  current_object = object[cur_row][cur_col][cur_arr]
	  
  subject = [
    [["slime",    "slime"], ["slime",    "door"], ["slime",    "sculptor"]],
    [["door",     "slime"], ["door",     "door"], ["door",     "sculptor"]],
    [["sculptor", "slime"], ["scultpor", "door"], ["sculptor", "sculptor"]],
  ]

  row_options = [0, 1, 2]
  col_options = [0, 1, 2]
  arr_options = [0, 1]
	  
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample
	  
  current_subject = subject[cur_row][cur_col][cur_arr]
      	  
  verb = [
    [["kill",   "kill"], ["kill",   "unlock"], ["kill",   "touch"]],
    [["unlock", "kill"], ["unlock", "unlock"], ["unlock", "touch"]],
    [["touch",  "kill"], ["touch",  "unlock"], ["touch",  "touch"]],
  ]

  row_options = [0, 1, 2]
  col_options = [0, 1, 2]
  arr_options = [0, 1]
	  
  cur_row = row_options.sample
  cur_col = col_options.sample
  cur_arr = arr_options.sample
	  
  current_verb = verb[cur_row][cur_col][cur_arr]
	  
  File.open("data/behehaviours/enemies.txt", "a") { |f|
    f.puts "#{current_object} #{current_subject} #{current_verb}"
  }
end

def self.learn_behaviours ## This routine is loosely inspired by the Tesla Vortex.
  #     1    2    4    8    7    5
  # 1 1,1  1,2  1,4  1,8  1,7  1,5
  # 2 2,1  2,2  2,4  2,8  2,7  2,5
  # 4 4,1  4,2  4,4  4,8  4,7  4,5
  # 8 8,1  8,2  8,4  8,8  8,7  8,5
  # 7 7,1  7,2  7,4  7,8  7,7  7,5
  # 5 5,1  5,2  5,4  5,8  5,7  5,5

  #      3    6    9
  # 3  3,3  3,6  3,9
  # 6  6,3  6,6  6,9
  # 9  9,3  9,6  9,9

  vortex = [ 1, 2, 4, 8, 7, 5 ]
  charge = [ 3, 6, 9 ]

  nested_vortex = [
    [[1, 1], [1, 2], [1, 4], [1, 8], [1, 7], [1, 5]],
    [[2, 1], [2, 2], [2, 4], [2, 8], [2, 7], [2, 5]],
    [[4, 1], [4, 2], [4, 4], [4, 8], [4, 7], [4, 5]],
    [[8, 1], [8, 2], [8, 4], [8, 8], [8, 7], [8, 5]],
    [[7, 1], [7, 2], [7, 4], [7, 8], [7, 7], [7, 5]],
    [[5, 1], [5, 2], [5, 4], [5, 8], [5, 7], [5, 5]],
  ]

  nested_charge = [
    [[3, 3], [3, 6], [3, 9]],
    [[6, 3], [6, 6], [6, 9]],
    [[9, 3], [9, 6], [9, 9]],
  ]

  ## Choosing the first index value for vortex based loop.
  row_options_vortex = [0, 1, 2, 3, 4, 5]
  col_options_vortex = [0, 1, 2, 3, 4, 5]
  arr_options_vortex = [0, 1]

  v_cur_row = row_options_vortex.sample
  v_cur_col = col_options_vortex.sample
  v_cur_arr = arr_options_vortex.sample

  ## Choosing the first index value for charge based loop.
  row_options_charge = [0, 1, 2]
  col_options_charge = [0, 1, 2]
  arr_options_charge = [0, 1]

  c_cur_row = row_options_charge.sample
  c_cur_col = col_options_charge.sample
  c_cur_arr = arr_options_charge.sample

  chosen_vortex_value = nested_vortex[v_cur_row][v_cur_col][v_cur_arr]
  chosen_charge_value = nested_charge[c_cur_row][c_cur_col][c_cur_arr]

  puts chosen_vortex_value
  puts chosen_charge_value

  ## Vortex Loop
  chosen_vortex_value.times do 
    possible_lines = File.readlines("data/enemies/possible_behaviours.txt")
    size_limit     = possible_lines.size.to_i
    index          = 0 

    ideal_line = possible_lines[chosen_vortex_value].to_s

    size_limit.times do
      current_line = possible_lines[index].to_s

      if current_line == ideal_line
         File.open("lib/npc/tourguide/learned_lines.txt", "a") { |f|
           f.puts current_line
         }
      else
        #puts "> Current line did not match the ideal dialogue line..."
      end

      index = index + 1
    end

    index = 0
  end

  ## Charge Loop
  chosen_charge_value.times do
    learned_lines  = File.readlines("data/enemies/learned_behaviours.txt").shuffle
    size_limit     = learned_lines.size.to_i
    index          = 0 

    ideal_line = learned_lines[chosen_vortex_value].to_s

    size_limit.times do
      current_line = learned_lines[index].to_s

      if current_line == ideal_line
         puts current_line

         File.open("data/enemies/reinforced_behaviours.txt", "a") { |f|
           f.puts current_line
         }
      else
        #puts "> Current line is not reinforced into longterm memory..."
      end

      index = index + 1
    end

    index = 0
  end
end

def self.cnap # Choose Next Actions From Analyzed Patterns
  require "SelfModifiedDecisionTree"
	  
  behavioural_models = RevisedBayes.new(:key,   :magic_stone,    :sword), # Object
                       RevisedBayes.new(:slime,        :door, :sculptor), # Subject
		       RevisedBayes.new(:kill,       :unlock,    :touch), # Verb

  object_type  = behavioural_models[0]
  subject_type = behavioural_models[1]
  verb_type    = behavioural_models[2]
	  
  ## Object Types
  object_type.train(:key,                    "key door unlock",         "key")
  object_type.train(:sword,                 "sword slime kill",       "sword")
  object_type.train(:magic_stone, "magic stone sculptor touch", "magic stone")
	  
  ## Subject Types
  subject_type.train(:slime,               "key door unlock",    "slime")
  subject_type.train(:door,               "sword slime kill",     "door")
  subject_type.train(:sculptor, "magic stone sculptor touch", "sculptor")
	  
  ## Verb Types
  verb_type.train(:kill,              "key door unlock",   "kill")
  verb_type.train(:unlock,           "sword slime kill", "unlock")
  verb_type.train(:touch,  "magic stone sculptor touch",  "touch")
	  
  learned_behaviours = File.readlines("data/enemies/learned_behaviours.txt")
  size_limit         = learned_behaviours.size.to_i
  input              = 0
	  
  size_limit.times do
    object_classification  = object_type.classift(learned_behaviours[input])
    subject_classification = subject_type.classift(learned_behaviours[input])
    verb_classification    = verb_type.classift(learned_behaviours[input])
		
    attributes = ["Objects"], ["Subjects"], ["Verbs"]
		
    training = [
      [0.005,         "key"],
      [0.502,       "sword"],
      [0.999, "magic stone"],
    ], [
      [0.005,    "slime"],
      [0.502,     "door"],
      [0.999, "sculptor"],
    ], [
      [0.005,   "kill"],
      [0.502, "unlock"],
      [0.999,  "touch"],
    ]

    dec_tree = DecisionTree::ID3Tree.new(attributes, training, 1, :continuous),
               DecisionTree::ID3Tree.new(attributes, training, 1, :continuous),
	       DecisionTree::ID3Tree.new(attributes, training, 1, :continuous)

    current_dectree1 = dec_tree[0]
    current_dectree1.train

    current_dectree2 = dec_tree[1]
    current_dectree2.train

    current_dectree3 = dec_tree[3]
    current_dectree3.train

    chosen_object  = [object_classification[1],   "sword"]
    chosen_subject = [subject_classification[1],   "door"]
    chosen_verb    = [verb_classification[1],    "unlock"]

    decision1 = dec_tree.predict(chosen_object)
    decision2 = dec_tree.predict(chosen_subject)
    decision3 = dec_tree.predict(chosen_verb)

    puts "Gribatomaton's Action: #{decision1} #{decision2} #{decision3}. #{object_classification} #{subject_classification} #{verb_classification}"
  end
end

################################################################################################
#                                      Word Generator                                          #
################################################################################################
def word_lengths
  def one_character
    dipthongs  = File.readlines("dictionary/dipthongs.txt")
    tripthongs = File.readlines("dictionary/tripthongs.txt")

    chosen_tripthong = tripthongs.sample.strip.to_s
    chosen_dipthong  = dipthongs.sample.strip.to_s

    generated_word = chosen_tripthong + chosen_dipthong

    masculine      = generated_word.chop +  "u"
    feminine       = generated_word.chop +  "a"
    plural         = generated_word.chop + "os"
    
    puts "Single Factor"
    puts "#{masculine} #{feminine} #{plural}"

    puts " "
  end

  def three_character
    dipthongs  = File.readlines("dictionary/dipthongs.txt")
    tripthongs = File.readlines("dictionary/tripthongs.txt")

    chosen_tripthong1 = tripthongs.sample.strip.to_s
    chosen_dipthong1  = dipthongs.sample.strip.to_s

    chosen_tripthong2 = tripthongs.sample.strip.to_s
    chosen_dipthong2  = dipthongs.sample.strip.to_s

    chosen_tripthong3 = tripthongs.sample.strip.to_s
    chosen_dipthong3  = dipthongs.sample.strip.to_s

    component_one = chosen_tripthong1 + chosen_dipthong1
    component_two = chosen_tripthong2 + chosen_dipthong2
    component_tre = chosen_tripthong3 + chosen_dipthong3

    generated_word = component_one + component_two + component_tre

    masculine      = generated_word.chop +  "u"
    feminine       = generated_word.chop +  "a"
    plural         = generated_word.chop + "os"
    
    puts "Three Factor"
    puts "#{masculine} #{feminine} #{plural}"

    puts " "
  end

  def five_character
    dipthongs  = File.readlines("dictionary/dipthongs.txt")
    tripthongs = File.readlines("dictionary/tripthongs.txt")

    chosen_tripthong1 = tripthongs.sample.strip.to_s
    chosen_dipthong1  = dipthongs.sample.strip.to_s

    chosen_tripthong2 = tripthongs.sample.strip.to_s
    chosen_dipthong2  = dipthongs.sample.strip.to_s

    chosen_tripthong3 = tripthongs.sample.strip.to_s
    chosen_dipthong3  = dipthongs.sample.strip.to_s

    chosen_tripthong4 = tripthongs.sample.strip.to_s
    chosen_dipthong4  = dipthongs.sample.strip.to_s

    chosen_tripthong5 = tripthongs.sample.strip.to_s
    chosen_dipthong5  = dipthongs.sample.strip.to_s

    component_one = chosen_tripthong1 + chosen_dipthong1
    component_two = chosen_tripthong2 + chosen_dipthong2
    component_tre = chosen_tripthong3 + chosen_dipthong3
    component_fro = chosen_tripthong4 + chosen_dipthong4
    component_fiv = chosen_tripthong5 + chosen_dipthong5

    generated_word = component_one + component_two + component_tre + component_fro + component_fiv

    masculine      = generated_word.chop +  "u"
    feminine       = generated_word.chop +  "a"
    plural         = generated_word.chop + "os"
    
    puts "Five Factor"
    puts "#{masculine} #{feminine} #{plural}"

    puts " "
  end

  def seven_character
    dipthongs  = File.readlines("dictionary/dipthongs.txt")
    tripthongs = File.readlines("dictionary/tripthongs.txt")

    chosen_tripthong1 = tripthongs.sample.strip.to_s
    chosen_dipthong1  = dipthongs.sample.strip.to_s

    chosen_tripthong2 = tripthongs.sample.strip.to_s
    chosen_dipthong2  = dipthongs.sample.strip.to_s

    chosen_tripthong3 = tripthongs.sample.strip.to_s
    chosen_dipthong3  = dipthongs.sample.strip.to_s

    chosen_tripthong4 = tripthongs.sample.strip.to_s
    chosen_dipthong4  = dipthongs.sample.strip.to_s

    chosen_tripthong5 = tripthongs.sample.strip.to_s
    chosen_dipthong5  = dipthongs.sample.strip.to_s

    chosen_tripthong6 = tripthongs.sample.strip.to_s
    chosen_dipthong6  = dipthongs.sample.strip.to_s

    chosen_tripthong7 = tripthongs.sample.strip.to_s
    chosen_dipthong7  = dipthongs.sample.strip.to_s

    component_one = chosen_tripthong1 + chosen_dipthong1 # dipthongs.sample.strip.to_s
    component_two = chosen_tripthong2 + chosen_dipthong2 # dipthongs.sample.strip.to_s
    component_tre = chosen_tripthong3 + chosen_dipthong3 # dipthongs.sample.strip.to_s
    component_fro = chosen_tripthong4 + chosen_dipthong4 # dipthongs.sample.strip.to_s
    component_fiv = chosen_tripthong5 + chosen_dipthong5 # dipthongs.sample.strip.to_s
    component_six = chosen_tripthong6 + chosen_dipthong6 # dipthongs.sample.strip.to_s
    component_sev = chosen_tripthong7 + chosen_dipthong7 # dipthongs.sample.strip.to_s

    generated_word = component_one + component_two + component_tre + component_fro + component_fiv + component_six + component_sev

    masculine      = generated_word.chop +  "u"
    feminine       = generated_word.chop +  "a"
    plural         = generated_word.chop + "os"
    
    puts "Seven Factor"
    puts "#{masculine} #{feminine} #{plural}"

    puts " "
  end

  def hybrid_compounds
    japanese_words = File.readlines("dictionary/japanese_words.txt")
    french_words   = File.readlines("dictionary/french_words.txt")

    current_nihongo  = japanese_words.sample.strip.to_s
    current_francais = french_words.sample.strip.to_s

    generated_word = current_nihongo + current_francais

    masculine_class = "Te "
    feminine_class  = "Ta "
    plural_class    = "Deso "

    generated_masculine_form = masculine_class + generated_word
    generated_feminine_form  = feminine_class  + generated_word
    generated_plural_form    = plural_class    + generated_word

    masculine_end_word = generated_masculine_form.chop + "u"
    feminine_end_word  = generated_feminine_form.chop + "a"
    plural_end_word    = generated_plural_form.chop + "os"

    puts "Naturalistic"
    puts "#{masculine_end_word}"
    puts "#{feminine_end_word}"
    puts "#{plural_end_word}"
  end
end

################################################################################################
#                                  Process Of Elimination                                      #
################################################################################################
def find_candidate
  ## Determining user data and user choice.
  value = File.read("_input/user/user_choice.txt").to_s.to_i

  user_data   = File.readlines("_data/user/candidates.txt")
  user_choice = user_data[value]

  ## Processing AI focused data
  ai_choice            = File.read("_data/ai/ai_choice.txt").to_s.to_i
  ai_initial_candidate = user_data[ai_choice]
  ai_search_limit      = user_data.size.to_i

  ## Create AI data from user data.
  ai_search_limit.times do
    if ai_choice == user_choice
      puts "The specific candidate was found. Terminating selection..."

      ai_data      = user_data.slice!(ai_choice)

      open("_data/ai/candidates.txt", "w") { |f|
        f.puts ai_data
      }
    else
      puts "The specific candidate is not found..."
    end
  end

  ## AI processing data.
  ai_choice            = File.read("_data/ai/ai_choice.txt").to_s.to_i
  ai_data              = File.readlines("_data/ai/candidates.txt")
  ai_search_limit      = ai_data.size.to_i
  ai_next_candidate    = ai_data[ai_choice]

  ai_search_limit.times do
    if ai_next_candidate == user_choice
      ai_final_candidate = ai_next_candidate
    
      puts "Candidate found, processing input..."; sleep(1)

      open("_posts/input.md", "w") { |f|
        f.puts "## #{date_title}"
        f.puts "By process of elimination, the bot chose: #{ai_final_candidate}"
      }
    else
      puts "Candidate is not yet found..."

      ai_choice            = File.read("_data/ai/ai_choice.txt").to_s.to_i
      ai_data              = File.readlines("_data/ai/candidates.txt")
      ai_search_limit      = ai_data.size.to_i
      ai_next_candidate    = ai_data[ai_choice]

      ai_data      = user_data.slice!(ai_choice)

      open("_data/ai/candidates.txt", "w") { |f|
        f.puts ai_data
      }
    end
  end
end

################################################################################################
#                                 Adversarial Decision Trees                                   #
################################################################################################
#######################################################################################################
#                            Competing And Coordinating Knowledge Bases                               #
#######################################################################################################
# This is an extension of selection that becomes a form of competing or competitive knowledge bases   #
# depending on what your goals are for training AI sy#
module SaadSelective
  class CoordinatedSelection
    def self.get_player_statistics(a1, a2, b1, b2, c1, c2)
      a = a1, a2
      b = b1, b2
      c = c1, c2

      matrix = [
        [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
        [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
        [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
      ], [
        [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
        [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
        [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
      ], [
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
      ]

      label_type       = matrix[0]
      definition_type  = matrix[1]
      probability_type = matrix[2]
  
      row_probability = 0.33
      col_probability = 0.33
  
      graph_selection = row_probability * col_probability

      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]

      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample
  
      current_label       = label_type[cur_row][cur_col][cur_arr]
      current_definition  = definition_type[cur_row][cur_col][cur_arr]
      current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
      puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
      @current_player_probability = current_probability + current_probability
      @current_player_information = "#{current_label} #{current_definition}"
      
      #puts "\n"
      #puts @current_player_probability
      
      #abort
    end
    
    def self.get_gribatomaton_statistics(a1, a2, b1, b2, c1, c2)
      a = a1, a2
      b = b1, b2
      c = c1, c2

      matrix = [
        [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
        [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
        [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
      ], [
        [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
        [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
        [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
      ], [
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
      ]

      label_type       = matrix[0]
      definition_type  = matrix[1]
      probability_type = matrix[2]
  
      row_probability = 0.33
      col_probability = 0.33
  
      graph_selection = row_probability * col_probability

      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]

      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample
  
      current_label       = label_type[cur_row][cur_col][cur_arr]
      current_definition  = definition_type[cur_row][cur_col][cur_arr]
      current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
      puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
      @current_gribatomaton_probability = current_probability + current_probability
      @current_gribatomaton_information = "#{current_label} #{current_definition}"
    end
    
    def self.get_enemy_statistics(a1, a2, b1, b2, c1, c2)
      a = a1, a2
      b = b1, b2
      c = c1, c2

      matrix = [
        [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
        [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
        [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
      ], [
        [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
        [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
        [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
      ], [
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
      ]

      label_type       = matrix[0]
      definition_type  = matrix[1]
      probability_type = matrix[2]
  
      row_probability = 0.33
      col_probability = 0.33
  
      graph_selection = row_probability * col_probability

      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]

      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample
  
      current_label       = label_type[cur_row][cur_col][cur_arr]
      current_definition  = definition_type[cur_row][cur_col][cur_arr]
      current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
      puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
      @current_enemy_probability = current_probability + current_probability
      @current_enemy_information = "#{current_label} #{current_definition}"
    end    
    
    #######################################################################################################
    #                                   Reconsideration And Reassessment                                  #
    #                                               For player                                            #
    #######################################################################################################
    def self.reasses_player
      #if @current_player_probability > 0.999999999999999999
        #@current_player_probability = 0.9 / @current_player_probability
      #end
      
      case @current_player_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_player_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_player_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_player_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_player_information} ]."
      else
        @current_player_probability = @current_player_probability + @current_player_probability
        
        reasses_player
      end
    end

    def self.reconsider_player    
      #if @current_player_probability > 0.999999999999999999
        #@current_player_probability = 0.9 / @current_player_probability
      #end
      
      case @current_player_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_player_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_player_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_player_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_player_information} ]."
      else
        @current_player_probability = @current_player_probability * @current_player_probability
        
        reconsider_player
      end
    end
    
    #######################################################################################################
    #                                   Reconsideration And Reassessment                                  #
    #                                           For gribatomaton                                          #
    #######################################################################################################
    def self.reasses_gribatomaton
      #if @current_gribatomaton_probability > 0.999999999999999999
        #@current_gribatomaton_probability = 0.9 / @current_gribatomaton_probability
      #end
      
      case @current_gribatomaton_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_gribatomaton_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_gribatomaton_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_gribatomaton_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_gribatomaton_information} ]."
      else
        @current_gribatomaton_probability = @current_gribatomaton_probability + @current_gribatomaton_probability
        
        reasses_gribatomaton
      end
    end

    def self.reconsider_gribatomaton
      #if @current_gribatomaton_probability > 0.999999999999999999
        #@current_gribatomaton_probability = 0.9 / @current_gribatomaton_probability
      #end
      
      case @current_gribatomaton_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_gribatomaton_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_gribatomaton_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_gribatomaton_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_gribatomaton_information} ]."
      else
        @current_gribatomaton_probability = @current_gribatomaton_probability * @current_gribatomaton_probability
        
        reconsider_gribatomaton
      end
    end
    
    #######################################################################################################
    #                                   Reconsideration And Reassessment                                  #
    #                                               For Enemey                                            #
    #######################################################################################################
    def self.reasses_enemy
      #if @current_enemy_probability > 0.999999999999999999
        #@current_enemy_probability = 0.9 / @current_enemy_probability
      #end
      
      case @current_enemy_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_enemy_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_enemy_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_enemy_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_enemy_information} ]."
      else
        @current_enemy_probability = @current_enemy_probability + @current_enemy_probability
        
        reasses_enemy
      end
    end

    def self.reconsider_enemy
      #if @current_enemy_probability > 0.999999999999999999
        #@current_enemy_probability = 0.9 / @current_enemy_probability
      #end
      
      case @current_enemy_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_enemy_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_enemy_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_enemy_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_enemy_information} ]."
      else
        @current_enemy_probability = @current_enemy_probability * @current_enemy_probability
        
        reconsider_enemy
      end
    end
    
    def self.current_information
      print @current_player_information
      puts @current_player_probability
      
      print @current_gribatomaton_information
      puts @current_gribatomaton_probability
      
      print @current_enemy_information
      puts @current_enemy_probability
    end
  
    def self.decrement_confidence # Input taxation
      if    @current_player_probability < 0.50; # Lose HP
        if    @current_gribatomaton_probability    > @current_enemy_probability;           puts SaadSelective::CompetitiveSelection.reasses_enemy;        puts SaadSelective::CompetitiveSelection.reasses_gribatomaton
        elsif @current_enemy_probability           > @current_gribatomaton_probability;    puts SaadSelective::CompetitiveSelection.reasses_gribatomaton; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      elsif @current_player_probability > 0.75; # Gain HP
        if    @current_gribatomaton_probability    < @current_enemy_probability;        puts SaadSelective::CompetitiveSelection.reconsider_enemy;        puts SaadSelective::CompetitiveSelection.reasses_gribatomaton
        elsif @current_enemy_probability           < @current_gribatomaton_probability; puts SaadSelective::CompetitiveSelection.reconsider_gribatomaton; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      end
      
      #SaadSelective::CompetitiveSelection.current_information
      
      if    @current_gribatomaton_probability < 0.50;
        if    @current_player_probability  > @current_enemy_probability;   puts SaadSelective::CompetitiveSelection.reconsider_enemy;  puts SaadSelective::CompetitiveSelection.reasses_player
        elsif @current_enemy_probability   > @current_player_probability;  puts SaadSelective::CompetitiveSelection.reconsider_player; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      elsif @current_gribatomaton_probability > 0.75;
        if    @current_player_probability  > @current_enemy_probability;   puts SaadSelective::CompetitiveSelection.reconsider_enemy;  puts SaadSelective::CompetitiveSelection.reasses_player
        elsif @current_enemy_probability   > @current_player_probability;  puts SaadSelective::CompetitiveSelection.reconsider_player; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      end
      
      #SaadSelective::CompetitiveSelection.current_information

      if    @current_enemy_probability < 0.50;
        if    @current_player_probability       > @current_gribatomaton_probability; puts SaadSelective::CompetitiveSelection.reasses_gribatomaton; puts SaadSelective::CompetitiveSelection.reconsider_player
        elsif @current_gribatomaton_probability > @current_player_probability;       puts SaadSelective::CompetitiveSelection.reasses_player;       puts SaadSelective::CompetitiveSelection.reconsider_gribatomaton
        end
      elsif @current_enemy_probability > 0.75;
        if    @current_player_probability > @current_enemy_probability;              puts SaadSelective::CompetitiveSelection.reconsider_enemy;  puts SaadSelective::CompetitiveSelection.reasses_player
        elsif @current_enemy_probability  > @current_player_probability;             puts SaadSelective::CompetitiveSelection.reconsider_player; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      end
      
      #SaadSelective::CompetitiveSelection.current_information
      
      @current_player_probability       = @current_player_probability
      @current_gribatomaton_probability = @current_gribatomaton_probability
      @current_enemy_probability        = @current_enemy_probability
    end
  end
  
  class CompetitiveSelection
  
    def self.get_player_statistics(a1, a2, b1, b2, c1, c2)
      a = a1, a2
      b = b1, b2
      c = c1, c2

      matrix = [
        [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
        [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
        [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
      ], [
        [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
        [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
        [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
      ], [
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
      ]

      label_type       = matrix[0]
      definition_type  = matrix[1]
      probability_type = matrix[2]
  
      row_probability = 0.33
      col_probability = 0.33
  
      graph_selection = row_probability * col_probability

      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]

      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample
  
      current_label       = label_type[cur_row][cur_col][cur_arr]
      current_definition  = definition_type[cur_row][cur_col][cur_arr]
      current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
      puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
      @current_player_probability = current_probability + current_probability
      @current_player_information = "#{current_label} #{current_definition}"
    end
    
    def self.get_gribatomaton_statistics(a1, a2, b1, b2, c1, c2)
      a = a1, a2
      b = b1, b2
      c = c1, c2

      matrix = [
        [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
        [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
        [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
      ], [
        [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
        [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
        [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
      ], [
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
      ]

      label_type       = matrix[0]
      definition_type  = matrix[1]
      probability_type = matrix[2]
  
      row_probability = 0.33
      col_probability = 0.33
  
      graph_selection = row_probability * col_probability

      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]

      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample
  
      current_label       = label_type[cur_row][cur_col][cur_arr]
      current_definition  = definition_type[cur_row][cur_col][cur_arr]
      current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
      puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
      @current_gribatomaton_probability = current_probability + current_probability
      @current_gribatomaton_information = "#{current_label} #{current_definition}"
    end
    
    def self.get_enemy_statistics(a1, a2, b1, b2, c1, c2)
      a = a1, a2
      b = b1, b2
      c = c1, c2

      matrix = [
        [[a[0], a[0]], [a[0], b[0]], [a[0], c[0]]],
        [[b[0], a[0]], [b[0], b[0]], [b[0], c[0]]],
        [[c[0], a[0]], [c[0], b[0]], [c[0], c[0]]],
      ], [
        [[a[1], a[1]], [a[1], b[1]], [a[1], c[1]]],
        [[b[1], a[1]], [b[1], b[1]], [b[1], c[1]]],
        [[c[1], a[1]], [c[1], b[1]], [c[1], c[1]]],
      ], [
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
        [[0.50, 0.50], [0.50, 0.50], [0.50, 0.50]],
      ]

      label_type       = matrix[0]
      definition_type  = matrix[1]
      probability_type = matrix[2]
  
      row_probability = 0.33
      col_probability = 0.33
  
      graph_selection = row_probability * col_probability

      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]

      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample
  
      current_label       = label_type[cur_row][cur_col][cur_arr]
      current_definition  = definition_type[cur_row][cur_col][cur_arr]
      current_probability = probability_type[cur_row][cur_col][cur_arr] * graph_selection
  
      puts "I'm confident it is not [ #{current_label} #{current_definition} ] as it has only #{current_probability} probability."
  
      @current_enemy_probability = current_probability + current_probability
      @current_enemy_information = "#{current_label} #{current_definition}"
    end
    
    #######################################################################################################
    #                                   Reconsideration And Reassessment                                  #
    #                                             For Player                                              #
    #######################################################################################################
    def self.reasses_player
      #if @current_player_probability > 0.999999999999999999
        #@current_player_probability = 0.9 / @current_player_probability
      #end
  
      case @current_player_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_player_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_player_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_player_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_player_information} ]."
      else
        @current_player_probability = @current_player_probability + @current_player_probability
        
        reasses_player
      end
    end

    def self.reconsider_player
      #if @current_player_probability > 0.999999999999999999
        #@current_player_probability = 0.9 / @current_player_probability
      #end
  
      case @current_player_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_player_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_player_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_player_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_player_information} ]."
      else
        @current_player_probability = @current_player_probability * @current_player_probability
        
        reconsider_player
      end
      
    end
    
    #######################################################################################################
    #                                   Reconsideration And Reassessment                                  #
    #                                           For gribatomaton                                          #
    #######################################################################################################
    def self.reasses_gribatomaton
      #if @current_player_probability > 0.999999999999999999
        #@current_player_probability = 0.9 / @current_player_probability
      #end
  
      case @current_player_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_player_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_player_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_player_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_player_information} ]."
      else
        puts @current_player_probability = @current_player_probability + @current_player_probability
        
        reasses_gribatomaton
      end
    end

    def self.reconsider_gribatomaton
      #if @current_player_probability > 0.999999999999999999
        #@current_player_probability = 0.9 / @current_player_probability
      #end
  
      case @current_player_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_player_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_player_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_player_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_player_information} ]."
      else
        puts @current_player_probability = @current_player_probability * @current_player_probability
        
        reconsider_gribatomaton
      end
  
      #@current_player_probability = @current_player_probability * @current_player_probability
    end
    
    #######################################################################################################
    #                                   Reconsideration And Reassessment                                  #
    #                                               For Enemey                                            #
    #######################################################################################################
    def self.reasses_enemy
      #if @current_enemy_probability > 0.999999999999999999
        #@current_enemy_probability = 0.9 / @current_enemy_probability
      #end
  
      case @current_enemy_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_enemy_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_enemy_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_enemy_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_enemy_information} ]."
      else
        puts @current_player_probability = @current_player_probability + @current_player_probability
        
        reasses_enemy
      end
    end

    def self.reconsider_enemy
      #if @current_enemy_probability > 0.999999999999999999
        #@current_enemy_probability = 0.9 / @current_enemy_probability
      #end
  
      case @current_enemy_probability
      when 0.054450000000000005..0.287225000000000000
        puts "I'm confident it is not [ #{@current_enemy_information} ]."
      when 0.287225000000000001..0.522225000000000000
        puts "I'm less unconfident it is not [ #{@current_enemy_information} ]."
      when 0.522225000000000001..0.756112500000000000
        puts "I'm almost sure it is [ #{@current_enemy_information} ]."
      when 0.756112500000000001..0.999999999999999999
        puts "I'm sure it is [ #{@current_enemy_information} ]."
      else
        puts @current_player_probability = @current_player_probability * @current_player_probability
        
        reconsider_enemy
      end
    end
    
    
    def self.current_information
      print @current_player_information #.class
      puts @current_player_probability #.class
      
      print @current_gribatomaton_information #.class
      puts @current_gribatomaton_probability #.class
      
      print @current_enemy_information #.class
      puts @current_enemy_probability #.class
    end

    def self.increment_confidence # Input taxation
      #puts @current_player_probability.class
      #puts @current_gribatomaton_probability.class
      #puts @current_enemy_probability.class
    
      #abort
    
      if    @current_player_probability < 0.50; # Lose HP
        if    @current_gribatomaton_probability    > @current_enemy_probability;           puts SaadSelective::CoordinatedSelection.reconsider_enemy;        puts SaadSelective::CompetitiveSelection.reasses_gribatomaton
        elsif @current_enemy_probability           > @current_gribatomaton_probability;    puts SaadSelective::CoordinatedSelection.reconsider_gribatomaton; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      elsif @current_player_probability > 0.75; # Gain HP
        if    @current_gribatomaton_probability    < @current_enemy_probability;        puts SaadSelective::CoordinatedSelection.reconsider_gribatomaton; puts SaadSelective::CompetitiveSelection.reasses_enemy
        elsif @current_enemy_probability           < @current_gribatomaton_probability; puts SaadSelective::CoordinatedSelection.reconsider_enemy;        puts SaadSelective::CompetitiveSelection.reasses_gribatomaton
        end
      end
      
      #SaadSelective::CoordinatedSelection.current_information
      
      if    @current_gribatomaton_probability < 0.50;
        if    @current_player_probability  > @current_enemy_probability;   puts SaadSelective::CoordinatedSelection.reconsider_enemy;  puts SaadSelective::CompetitiveSelection.reasses_player
        elsif @current_enemy_probability   > @current_player_probability;  puts SaadSelective::CoordinatedSelection.reconsider_player; puts SaadSelective::CompetitiveSelection.reasses_enemy
        end
      elsif @current_gribatomaton_probability > 0.75;
        if    @current_player_probability  > @current_enemy_probability;   puts SaadSelective::CoordinatedSelection.reconsider_player; puts SaadSelective::CompetitiveSelection.reasses_enemy
        elsif @current_enemy_probability   > @current_player_probability;  puts SaadSelective::CoordinatedSelection.reconsider_enemy;  puts SaadSelective::CompetitiveSelection.reasses_player
        end
      end
      
      #SaadSelective::CoordinatedSelection.current_information

      if    @current_enemy_probability < 0.50;
        if    @current_player_probability       > @current_gribatomaton_probability; puts SaadSelective::CoordinatedSelection.reasses_player;       puts SaadSelective::CoordinatedSelection.reconsider_gribatomaton
        elsif @current_gribatomaton_probability > @current_player_probability;       puts SaadSelective::CoordinatedSelection.reasses_gribatomaton; puts SaadSelective::CoordinatedSelection.reconsider_player
        end
      elsif @current_enemy_probability > 0.75;
        if    @current_player_probability > @current_enemy_probability;              puts SaadSelective::CoordinatedSelection.reasses_player; puts SaadSelective::CoordinatedSelection.reconsider_enemy
        elsif @current_enemy_probability  > @current_player_probability;             puts SaadSelective::CoordinatedSelection.reasses_enemy;  puts SaadSelective::CoordinatedSelection.reconsider_player
        end
      end
      
      #SaadSelective::CoordinatedSelection.current_information
      
      @current_player_probability       = @current_player_probability
      @current_gribatomaton_probability = @current_gribatomaton_probability
      @current_enemy_probability        = @current_enemy_probability
    end
  end
end

module Saad
  class Competitive
    def self.specify_measurements(a, b, c, d, e, f)
      @player_mechanic        = a
      @player_attribute       = b
    
      @gribatomaton_mechanic  = c
      @gribatomaton_attribute = d

      @enemy_mechanic         = e
      @enemy_attribute        = f
    end
  
    def self.starting_stats
      #        04     06     07     09     10
      # 04  04,04  04,06  04,07  04,07  04,10
      # 06  06,04  06,06  06,07  06,07  06,10
      # 07  07,04  07,06  07,07  07,09  07,10
      # 09  09,04  09,06  09,07  09,09  09,10
      # 10  10,04  10,06  10,07  10,09  10,10

      starting_prediction = [
        [[ 0.4,  0.4], [ 0.4, 0.6], [ 0.4, 0.7], [ 0.4, 0.9], [ 0.4, 0.10]],
        [[ 0.6,  0.4], [ 0.6, 0.6], [ 0.6, 0.7], [ 0.6, 0.9], [ 0.6, 0.10]],
        [[ 0.7,  0.4], [ 0.7, 0.6], [ 0.7, 0.7], [ 0.7, 0.9], [ 0.7, 0.10]],
        [[ 0.9,  0.4], [ 0.9, 0.6], [ 0.9, 0.7], [ 0.9, 0.9], [ 0.9, 0.10]],
        [[ 0.10, 0.4], [0.10, 0.6], [0.10, 0.7], [0.10, 0.9], [0.10, 0.10]],
      ], [
        [[ 0.4,  0.4], [ 0.4, 0.6], [ 0.4, 0.7], [ 0.4, 0.9], [ 0.4, 0.10]],
        [[ 0.6,  0.4], [ 0.6, 0.6], [ 0.6, 0.7], [ 0.6, 0.9], [ 0.6, 0.10]],
        [[ 0.7,  0.4], [ 0.7, 0.6], [ 0.7, 0.7], [ 0.7, 0.9], [ 0.7, 0.10]],
        [[ 0.9,  0.4], [ 0.9, 0.6], [ 0.9, 0.7], [ 0.9, 0.9], [ 0.9, 0.10]],
        [[ 0.10, 0.4], [0.10, 0.6], [0.10, 0.7], [0.10, 0.9], [0.10, 0.10]],
      ], [
        [[ 0.4,  0.4], [ 0.4, 0.6], [ 0.4, 0.7], [ 0.4, 0.9], [ 0.4, 0.10]],
        [[ 0.6,  0.4], [ 0.6, 0.6], [ 0.6, 0.7], [ 0.6, 0.9], [ 0.6, 0.10]],
        [[ 0.7,  0.4], [ 0.7, 0.6], [ 0.7, 0.7], [ 0.7, 0.9], [ 0.7, 0.10]],
        [[ 0.9,  0.4], [ 0.9, 0.6], [ 0.9, 0.7], [ 0.9, 0.9], [ 0.9, 0.10]],
        [[ 0.10, 0.4], [0.10, 0.6], [0.10, 0.7], [0.10, 0.9], [0.10, 0.10]],
      ]

      ## Determines player starting hp
      player_row_options = [0, 1, 2, 3, 4]
      player_col_options = [0, 1, 2, 3, 4]
      player_arr_options = [0, 1]

      p_crow = player_row_options.sample
      p_ccol = player_col_options.sample
      p_carr = player_arr_options.sample

      @player_prediction = starting_prediction[0][p_crow][p_ccol][p_carr]

      ## Determines gribatomaton's starting hp
      gribatomaton_row_options = [0, 1, 2, 3, 4]
      gribatomaton_col_options = [0, 1, 2, 3, 4]
      gribatomaton_arr_options = [0, 1]

      g_crow = gribatomaton_row_options.sample
      g_ccol = gribatomaton_col_options.sample
      g_carr = gribatomaton_arr_options.sample

      @gribatomaton_prediction = starting_prediction[1][g_crow][g_ccol][g_carr]

      ## Determines enemy starting hp
      enemy_row_options = [0, 1, 2, 3, 4]
      enemy_col_options = [0, 1, 2, 3, 4]
      enemy_arr_options = [0, 1]

      e_crow = enemy_row_options.sample
      e_ccol = enemy_col_options.sample
      e_carr = enemy_arr_options.sample

      @enemy_prediction = starting_prediction[2][e_crow][e_ccol][e_carr]
    end
  
    def self.evaluate_player
      require "SelfModifiedDecisionTree"

      attribute = ["Player"]
    
      training = [
        [0.0010,    "Some #{@player_attribute} #{@player_mechanic}"],
        [0.2505,    "Mild #{@player_attribute} #{@player_mechanic}"],
        [0.5000,  "Medium #{@player_attribute} #{@player_mechanic}"],
        [0.7495,    "High #{@player_attribute} #{@player_mechanic}"],
        [0.9990,     "Max #{@player_attribute} #{@player_mechanic}"],
      ]

      dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute, training, 1, :continuous)

      current_dectree1 = dec_tree_configurations
      current_dectree1.train

      test1 = [@player_prediction, "Medium #{@player_attribute}"]

      @decision = current_dectree1.predict(test1)

      puts "Predicted: #{@decision}"

      #@net_outcome1 = "Predicted: #{@decision}"
    end
  
    def self.evaluate_gribatomaton
      require "SelfModifiedDecisionTree"
    
      attribute = ["Gribatomaton"]
    
      training = [
        [0.0010,    "Some #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.2505,    "Mild #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.5000,  "Medium #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.7495,    "High #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.9990,     "Max #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
      ]

      dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute, training, 1, :continuous)

      current_dectree1 = dec_tree_configurations
      current_dectree1.train

      test1 = [@gribatomaton_prediction, "Medium #{@gribatomaton_attribute}"]

      @decision = current_dectree1.predict(test1)

      puts "Predicted: #{@decision}"

      #@net_outcome2 = "Predicted: #{@decision}"
    end
  
    def self.evaluate_enemy
      require "SelfModifiedDecisionTree"
    
      attribute = ["Enemy"]
    
      training = [
        [0.0010,    "Some #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.2505,    "Mild #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.5000,  "Medium #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.7495,    "High #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.9990,     "Max #{@enemy_attribute} #{@enemy_mechanic}"],
      ]

      dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute, training, 1, :continuous)

      current_dectree1 = dec_tree_configurations
      current_dectree1.train

      test1 = [@enemy_prediction, "Medium #{@enemy_attribute}"]

      @decision = current_dectree1.predict(test1)

      puts "Predicted: #{@decision}"

      #@net_outcome3 = "Predicted: #{@decision}"
    end
  
    def self.mechanic_prediction
      matrix = [
        [[@player_mechanic,       @player_mechanic], [@player_mechanic,       @gribatomaton_mechanic], [@player_mechanic,       @enemy_mechanic]],
        [[@gribatomaton_mechanic, @player_mechanic], [@gribatomaton_mechanic, @gribatomaton_mechanic], [@gribatomaton_mechanic, @enemy_mechanic]],
        [[@enemy_mechanic,        @player_mechanic], [@enemy_mechanic,        @gribatomaton_mechanic], [@enemy_mechanic,        @enemy_mechanic]],
      ], [
        [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]],
        [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]],
        [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]],
      ]

      row_probability = 0.33
      col_probability = 0.33

      selection_probability = row_probability * col_probability
 
      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]
      
      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample

      dice_roll = matrix[0][cur_row][cur_col][cur_arr], matrix[1][cur_row][cur_col][cur_arr]
  
      d1 = dice_roll[0]
      d2 = dice_roll[1]

      puts "[#{d1}, #{selection_probability}], For Row #{cur_row} And #{cur_col}"
    
      @overall_prediction = selection_probability * d2 # selection_probability
    
      #puts @overall_prediction
    end
  
    def self.increment_inputs # Input taxation
      player_input       = @player_prediction
      gribatomaton_input = @gribatomaton_prediction
      enemy_input        = @enemy_prediction

      if    player_input < 50.0; # Lose HP
        if    gribatomaton_input    > enemy_input;           player_input = player_input - gribatomaton_input
        elsif enemy_input           > gribatomaton_input;    player_input = player_input - enemy_input
        end
      elsif player_input > 75.0; # Gain HP
        if    gribatomaton_input    < enemy_input;           player_input = player_input + gribatomaton_input
        elsif enemy_input           < gribatomaton_input;    player_input = player_input + gribatomaton_input
        end
      end

      if    gribatomaton_input < 50.0;
        if    player_input  > enemy_input;   gribatomaton_input = gribatomaton_input - player_input
        elsif enemy_input   > player_input;  gribatomaton_input = gribatomaton_input - enemy_input
        end
      elsif gribatomaton_input > 75.0;
        if    player_input  > enemy_input; gribatomaton_input = gribatomaton_input - player_input
        elsif enemy_input > player_input;  gribatomaton_input = gribatomaton_input - enemy_input
        end
      end

      if    enemy_input < 50.0;
        if    player_input  > enemy_input;   enemy_input = enemy_input - player_input
        elsif enemy_input   > player_input;  enemy_input = enemy_input - enemy_input
        end
      elsif enemy_input > 75.0;
        if    player_input > enemy_input;   enemy_input = enemy_input + player_input
        elsif enemy_input  > player_input;  enemy_input = enemy_input + enemy_input
        end
      end

      @player_prediction       = player_input
      @gribatomaton_prediction = gribatomaton_input
      @enemy_prediction        = enemy_input
    end
  end
  
  class Coordinated
    def self.specify_measurements(a, b, c, d, e, f)
      @player_mechanic        = a
      @player_attribute       = b
    
      @gribatomaton_mechanic  = c
      @gribatomaton_attribute = d

      @enemy_mechanic         = e
      @enemy_attribute        = f
    end
  
    def self.starting_stats
      #        04     06     07     09     10
      # 04  04,04  04,06  04,07  04,07  04,10
      # 06  06,04  06,06  06,07  06,07  06,10
      # 07  07,04  07,06  07,07  07,09  07,10
      # 09  09,04  09,06  09,07  09,09  09,10
      # 10  10,04  10,06  10,07  10,09  10,10

      starting_prediction = [
        [[ 0.4,  0.4], [ 0.4, 0.6], [ 0.4, 0.7], [ 0.4, 0.9], [ 0.4, 0.10]],
        [[ 0.6,  0.4], [ 0.6, 0.6], [ 0.6, 0.7], [ 0.6, 0.9], [ 0.6, 0.10]],
        [[ 0.7,  0.4], [ 0.7, 0.6], [ 0.7, 0.7], [ 0.7, 0.9], [ 0.7, 0.10]],
        [[ 0.9,  0.4], [ 0.9, 0.6], [ 0.9, 0.7], [ 0.9, 0.9], [ 0.9, 0.10]],
        [[ 0.10, 0.4], [0.10, 0.6], [0.10, 0.7], [0.10, 0.9], [0.10, 0.10]],
      ], [
        [[ 0.4,  0.4], [ 0.4, 0.6], [ 0.4, 0.7], [ 0.4, 0.9], [ 0.4, 0.10]],
        [[ 0.6,  0.4], [ 0.6, 0.6], [ 0.6, 0.7], [ 0.6, 0.9], [ 0.6, 0.10]],
        [[ 0.7,  0.4], [ 0.7, 0.6], [ 0.7, 0.7], [ 0.7, 0.9], [ 0.7, 0.10]],
        [[ 0.9,  0.4], [ 0.9, 0.6], [ 0.9, 0.7], [ 0.9, 0.9], [ 0.9, 0.10]],
        [[ 0.10, 0.4], [0.10, 0.6], [0.10, 0.7], [0.10, 0.9], [0.10, 0.10]],
      ], [
        [[ 0.4,  0.4], [ 0.4, 0.6], [ 0.4, 0.7], [ 0.4, 0.9], [ 0.4, 0.10]],
        [[ 0.6,  0.4], [ 0.6, 0.6], [ 0.6, 0.7], [ 0.6, 0.9], [ 0.6, 0.10]],
        [[ 0.7,  0.4], [ 0.7, 0.6], [ 0.7, 0.7], [ 0.7, 0.9], [ 0.7, 0.10]],
        [[ 0.9,  0.4], [ 0.9, 0.6], [ 0.9, 0.7], [ 0.9, 0.9], [ 0.9, 0.10]],
        [[ 0.10, 0.4], [0.10, 0.6], [0.10, 0.7], [0.10, 0.9], [0.10, 0.10]],
      ]

      ## Determines player starting hp
      player_row_options = [0, 1, 2, 3, 4]
      player_col_options = [0, 1, 2, 3, 4]
      player_arr_options = [0, 1]

      p_crow = player_row_options.sample
      p_ccol = player_col_options.sample
      p_carr = player_arr_options.sample

      @player_prediction = starting_prediction[0][p_crow][p_ccol][p_carr]

      ## Determines gribatomaton's starting hp
      gribatomaton_row_options = [0, 1, 2, 3, 4]
      gribatomaton_col_options = [0, 1, 2, 3, 4]
      gribatomaton_arr_options = [0, 1]

      g_crow = gribatomaton_row_options.sample
      g_ccol = gribatomaton_col_options.sample
      g_carr = gribatomaton_arr_options.sample

      @gribatomaton_prediction = starting_prediction[1][g_crow][g_ccol][g_carr]

      ## Determines enemy starting hp
      enemy_row_options = [0, 1, 2, 3, 4]
      enemy_col_options = [0, 1, 2, 3, 4]
      enemy_arr_options = [0, 1]

      e_crow = enemy_row_options.sample
      e_ccol = enemy_col_options.sample
      e_carr = enemy_arr_options.sample

      @enemy_prediction = starting_prediction[2][e_crow][e_ccol][e_carr]
    end
  
    def self.evaluate_player
      require "SelfModifiedDecisionTree"

      attribute = ["Player"]
    
      training = [
        [0.0010,    "Some #{@player_attribute} #{@player_mechanic}"],
        [0.2505,    "Mild #{@player_attribute} #{@player_mechanic}"],
        [0.5000,  "Medium #{@player_attribute} #{@player_mechanic}"],
        [0.7495,    "High #{@player_attribute} #{@player_mechanic}"],
        [0.9990,     "Max #{@player_attribute} #{@player_mechanic}"],
      ]

      dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute, training, 1, :continuous)

      current_dectree1 = dec_tree_configurations
      current_dectree1.train

      test1 = [@player_prediction, "Medium #{@player_attribute}"]

      @decision = current_dectree1.predict(test1)

      puts "Predicted: #{@decision}"

      #@net_outcome1 = "Predicted: #{@decision}"
    end
  
    def self.evaluate_gribatomaton
      require "SelfModifiedDecisionTree"
    
      attribute = ["Gribatomaton"]
    
      training = [
        [0.0010,    "Some #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.2505,    "Mild #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.5000,  "Medium #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.7495,    "High #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
        [0.9990,     "Max #{@gribatomaton_attribute} #{@gribatomaton_mechanic}"],
      ]

      dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute, training, 1, :continuous)

      current_dectree1 = dec_tree_configurations
      current_dectree1.train

      test1 = [@gribatomaton_prediction, "Medium #{@gribatomaton_attribute}"]

      @decision = current_dectree1.predict(test1)

      puts "Predicted: #{@decision}"

      #@net_outcome2 = "Predicted: #{@decision}"
    end
  
    def self.evaluate_enemy
      require "SelfModifiedDecisionTree"
    
      attribute = ["Enemy"]
    
      training = [
        [0.0010,    "Some #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.2505,    "Mild #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.5000,  "Medium #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.7495,    "High #{@enemy_attribute} #{@enemy_mechanic}"],
        [0.9990,     "Max #{@enemy_attribute} #{@enemy_mechanic}"],
      ]

      dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute, training, 1, :continuous)

      current_dectree1 = dec_tree_configurations
      current_dectree1.train

      test1 = [@enemy_prediction, "Medium #{@enemy_attribute}"]

      @decision = current_dectree1.predict(test1)

      puts "Predicted: #{@decision}"

      #@net_outcome3 = "Predicted: #{@decision}"
    end
  
    def self.mechanic_prediction
      matrix = [
        [[@player_mechanic,       @player_mechanic], [@player_mechanic,       @gribatomaton_mechanic], [@player_mechanic,       @enemy_mechanic]],
        [[@gribatomaton_mechanic, @player_mechanic], [@gribatomaton_mechanic, @gribatomaton_mechanic], [@gribatomaton_mechanic, @enemy_mechanic]],
        [[@enemy_mechanic,        @player_mechanic], [@enemy_mechanic,        @gribatomaton_mechanic], [@enemy_mechanic,        @enemy_mechanic]],
      ], [
        [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]],
        [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]],
        [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]],
      ]

      row_probability = 0.33
      col_probability = 0.33

      selection_probability = row_probability * col_probability
 
      row_options = [0, 1, 2]
      col_options = [0, 1, 2]
      arr_options = [0, 1]
      
      cur_row = row_options.sample
      cur_col = col_options.sample
      cur_arr = arr_options.sample

      dice_roll = matrix[0][cur_row][cur_col][cur_arr], matrix[1][cur_row][cur_col][cur_arr]
  
      d1 = dice_roll[0]
      d2 = dice_roll[1]

      puts "[#{d1}, #{selection_probability}], For Row #{cur_row} And #{cur_col}"
    
      @overall_prediction = selection_probability * d2 # selection_probability
    
      #puts @overall_prediction
    end
  
    def self.decrement_inputs # Input taxation
      player_input       = @player_prediction
      gribatomaton_input = @gribatomaton_prediction
      enemy_input        = @enemy_prediction

      if    player_input < 50.0; # Lose HP
        if    gribatomaton_input    > enemy_input;           player_input = player_input + gribatomaton_input
        elsif enemy_input           > gribatomaton_input;    player_input = player_input + enemy_input
        end
      elsif player_input > 75.0; # Gain HP
        if    gribatomaton_input    < enemy_input;           player_input = player_input - gribatomaton_input
        elsif enemy_input           < gribatomaton_input;    player_input = player_input - gribatomaton_input
        end
      end

      if    gribatomaton_input < 50.0;
        if    player_input  > enemy_input;   gribatomaton_input = gribatomaton_input + player_input
        elsif enemy_input   > player_input;  gribatomaton_input = gribatomaton_input + enemy_input
        end
      elsif gribatomaton_input > 75.0;
        if    player_input  > enemy_input; gribatomaton_input = gribatomaton_input + player_input
        elsif enemy_input > player_input;  gribatomaton_input = gribatomaton_input + enemy_input
        end
      end

      if    enemy_input < 50.0;
        if    player_input  > enemy_input;   enemy_input = enemy_input + player_input
        elsif enemy_input   > player_input;  enemy_input = enemy_input + enemy_input
        end
      elsif enemy_input > 75.0;
        if    player_input > enemy_input;   enemy_input = enemy_input - player_input
        elsif enemy_input  > player_input;  enemy_input = enemy_input - enemy_input
        end
      end

      @player_prediction       = player_input
      @gribatomaton_prediction = gribatomaton_input
      @enemy_prediction        = enemy_input
    end
  end
end

################################################################################################
#                                   Remembered Fortunes                                        #
################################################################################################
def fortune_generation
  iteration = File.read("_ai/gen_limit/limitation.txt").strip.to_i

  iteration.times do
    # Make fortunes cookies by two future array clones.
    fortune_cookie1 = File.readlines("_ai/fortunes/set_one/futures.txt")
    fortune_cookie2 = File.readlines("_ai/fortunes/set_two/futures.txt")

    # Use sampler for each fortune cookie
    first_fortune  = fortune_cookie1.sample.strip
    second_fortune = fortune_cookie2.sample.strip
    outcome_name   = "result".tr " ", "_"

    if first_fortune == second_fortune
      puts "First fortune: #{first_fortune} Second fortune: #{second_fortune}"

      puts ">> These outcomes are similar and thus remembering these for later..."

      ## Create a new document based on remembered datapoint
      open("_outcomes/remembered_futures/fortune.txt", "a") { |f|
        f.puts first_fortune
      }

      ## Create a new prolog knowledge base section upon appending.
      open("_knowledgebase/#{outcome_name}.pl", "a") { |f|
        f.puts first_fortune
      }

      abort
    else
      puts "First fortune: #{first_fortune} Second fortune: #{second_fortune}"

      puts ">> These outcomes are not similar and thus must be unconnected..." 
    end
  end
end

def retrain_fortune
  new_fortune = File.read("_outcomes/remembered_futures/fortune.txt")

  open("_ai/fortunes/set_one/futures.txt", "w") { |f|
    f.puts new_fortune
  }

  open("_ai/fortunes/set_two/futures.txt", "w") { |f|
    f.puts new_fortune
  }

  puts ">> Reset fortune list for each set as remembered futures."
end

def reset_limitation
  fortune_cookie1 = File.readlines("_ai/fortunes/set_one/futures.txt")

  loop_resizer = fortune_cookie1.size.to_i

  open("_ai/gen_limit/limitation.txt", "w") { |f|
    f.puts loop_resizer
  }
end

################################################################################################
#                               Procedural Dialogue Generation                                 #
################################################################################################
def self.generate_dialogue
  word_classes = [
    [
      [["Le",  "Le"], ["Le",  "La"], ["Le",  "Les"]],
      [["La",  "Le"], ["La",  "La"], ["La",  "Les"]],
      [["Les", "Le"], ["Les", "La"], ["Les", "Les"]],
    ], [
      [["Anu",  "Anu"], ["Anu",  "Ana"], ["Anu",  "Anos"]],
      [["Ana",  "Anu"], ["Ana",  "Ana"], ["Ana",  "Anos"]],
      [["Anos", "Anu"], ["Anos", "Ana"], ["Anos", "Anus"]],
    ], [
      [["Lanu",  "Lanu"], ["Lanu",  "Lana"], ["Lanu",  "Lanos"]],
      [["Lana",  "Lanu"], ["Lana",  "Lana"], ["Lana",  "Lanos"]],
      [["Lanos", "Lanu"], ["Lanos", "Lana"], ["Lanos", "Lanos"]],
    ],
  ]

  context_window = [0, 1, 2]
  row_options    = [0, 1, 2]
  col_options    = [0, 1, 2]
  arr_options    = [0, 1]

  cur_con = context_window.sample
  cur_row =    row_options.sample
  cur_col =    col_options.sample
  cur_arr =    arr_options.sample

  @current_word_class = word_classes[cur_con][cur_row][cur_col][cur_arr]
  #print current_word_class; print " "

  ho   = "homme"
  fe   = "femme"
  fi   = "fille"
  ga   = "garcon"
  ta   = "tante"
  oj   = "oncle"
  cofi = "cousinfille"
  coga = "cousingarcon"
  grm  = "grandmere"
  grp  = "grandpere"

  ct   = "chat"
  ch   = "chien"
  oi   = "oiseau"
  gr   = "souris"
  ou   = "ours"
  wo   = "orgueil"
  pr   = "ostritch"
  po   = "jiraff"
  pi   = "écureuil"

  m    = "maison"
  c    = "cabin"
  e    = "ecole"

  oju  = "ojijaku"
  neo  = "ne ojijaku"

  nouns = [
    [[ho,   ho], [ho,   fe], [ho,   fi], [ho,   ga], [ho,   ta], [ho,   oj], [ho,   cofi], [ho,   coga], [ho,   grm], [ho,   grp]],
    [[fe,   ho], [fe,   fe], [fe,   fi], [fe,   ga], [fe,   ta], [fe,   oj], [fe,   cofi], [fe,   coga], [fe,   grm], [fe,   grp]],
    [[fi,   ho], [fi,   fe], [fi,   fi], [fi,   ga], [fi,   ta], [fi,   oj], [fi,   cofi], [fi,   coga], [fi,   grm], [fi,   grp]],
    [[ga,   ho], [ga,   fe], [ga,   fi], [ga,   ga], [ga,   ta], [ga,   oj], [ga,   cofi], [ga,   coga], [ga,   grm], [ga,   grp]],
    [[ta,   ho], [ta,   fe], [ta,   fi], [ta,   ga], [ta,   ta], [ta,   oj], [ta,   cofi], [ta,   coga], [ta,   grm], [ta,   grp]],
    [[oj,   ho], [oj,   fe], [oj,   fi], [oj,   ga], [oj,   ta], [oj,   oj], [oj,   cofi], [oj,   coga], [oj,   grm], [oj,   grp]],
    [[cofi, ho], [cofi, fe], [cofi, fi], [cofi, ga], [cofi, ta], [cofi, oj], [cofi, cofi], [cofi, coga], [cofi, grm], [cofi, grp]],
    [[coga, ho], [coga, fe], [coga, fi], [coga, ga], [coga, ta], [coga, oj], [coga, cofi], [coga, coga], [coga, grm], [coga, grp]],
    [[grm,  ho], [grm,  fe], [grm,  fi], [grm,  ga], [grm,  ta], [grm,  oj], [grm,  cofi], [grm,  coga], [grm,  grm], [grm,  grp]],
    [[grp,  ho], [grp,  fe], [grp,  fi], [grp,  ga], [grp,  ta], [grp,  oj], [grp,  cofi], [grp,  coga], [grp,  grm], [grp,  grp]],
  ], [
    [[ct, ct], [ct, ch], [ct, oi], [ct, gr], [ct, wo], [ct, ou], [ct, pr], [ct, po]],
    [[ch, ct], [ch, ch], [ch, oi], [ch, gr], [ch, wo], [ch, ou], [ch, pr], [ch, po]],
    [[oi, ct], [oi, ch], [oi, oi], [pi, gr], [oi, wo], [oi, ou], [oi, pr], [oi, po]],
    [[gr, ct], [gr, ch], [gr, oi], [gr, gr], [gr, wo], [gr, ou], [gr, pr], [gr, po]],
  ], [
    [[m, m], [m, c], [m, e]],
    [[c, m], [c, c], [c, e]],
    [[e, m], [e, c], [e, e]],
  ], [
    [[oju, oju], [oju, neo]],
    [[neo, oju], [neo, neo]],
  ]

  context_window = [0, 1, 2, 3]
  cur_con        = context_window.sample

  if    cur_con == 0
    row_options    = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    col_options    = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    arr_options    = [0, 1]

    #cur_con = context_window.sample
    cur_row =    row_options.sample
    cur_col =    col_options.sample
    cur_arr =    arr_options.sample

    @current_noun = nouns[cur_con][cur_row][cur_col][cur_arr]
    #print current_noun; print " "

  elsif cur_con == 1
    row_options    = [0, 1, 2, 3]
    col_options    = [0, 1, 2, 3]
    arr_options    = [0, 1]

    #cur_con = context_window.sample
    cur_row =    row_options.sample
    cur_col =    col_options.sample
    cur_arr =    arr_options.sample

    @current_noun = nouns[cur_con][cur_row][cur_col][cur_arr]
    #print current_noun; print " "

  elsif cur_con == 2
    row_options    = [0, 1, 2]
    col_options    = [0, 1, 2]
    arr_options    = [0, 1]

    #cur_con = context_window.sample
    cur_row =    row_options.sample
    cur_col =    col_options.sample
    cur_arr =    arr_options.sample

    @current_noun = nouns[cur_con][cur_row][cur_col][cur_arr]

  elsif cur_con == 3
    row_options    = [0, 1]
    col_options    = [0, 1]
    arr_options    = [0, 1]

    #cur_con = context_window.sample
    cur_row =    row_options.sample
    cur_col =    col_options.sample
    cur_arr =    arr_options.sample

    @current_noun = nouns[cur_con][cur_row][cur_col][cur_arr]
    #print current_noun; print " "

  end

  subjects = [
    [["es",    "es"], ["es",    "es ne"]],
    [["es ne", "es"], ["es ne", "es ne"]],
  ], [
    [["es",    "es"], ["es",    "es ne"]],
    [["es ne", "es"], ["es ne", "es ne"]],
  ]

  context_window = [0, 1]

  row_options    = [0, 1]
  col_options    = [0, 1]
  arr_options    = [0, 1]

  cur_con = context_window.sample
  cur_row =    row_options.sample
  cur_col =    col_options.sample
  cur_arr =    arr_options.sample

  @current_subject = subjects[cur_con][cur_row][cur_col][cur_arr]
  #print current_subject; print " "

  avo = "avoir"
  cou = "coupe"
  ser = "serrure"
  dev = "déverrouillage"

  verbs = [
    [[avo, avo], [avo, cou], [avo, ser], [avo, dev]],
    [[cou, avo], [cou, cou], [cou, ser], [cou, dev]],
    [[ser, avo], [ser, cou], [ser, ser], [ser, dev]],
    [[dev, avo], [dev, cou], [dev, ser], [dev, dev]],
  ], [
    [[cou, avo], [cou, cou], [cou, ser], [cou, dev]],
    [[ser, avo], [ser, cou], [ser, ser], [ser, dev]],
    [[dev, avo], [dev, cou], [dev, ser], [dev, dev]],
    [[avo, avo], [avo, cou], [avo, ser], [avo, dev]],
  ], [
    [[ser, avo], [ser, cou], [ser, ser], [ser, dev]],
    [[dev, avo], [dev, cou], [dev, ser], [dev, dev]],
    [[avo, avo], [avo, cou], [avo, ser], [avo, dev]],
    [[cou, avo], [cou, cou], [cou, ser], [cou, dev]],
  ], [
    [[dev, avo], [dev, cou], [dev, ser], [dev, dev]],
    [[avo, avo], [avo, cou], [avo, ser], [avo, dev]],
    [[cou, avo], [cou, cou], [cou, ser], [cou, dev]],
    [[ser, avo], [ser, cou], [ser, ser], [ser, dev]],
  ]

  context_window = [0, 1, 2, 3]
  row_options    = [0, 1, 2, 3]
  col_options    = [0, 1, 2, 3]
  arr_options    = [0, 1]

  cur_con = context_window.sample
  cur_row =    row_options.sample
  cur_col =    col_options.sample
  cur_arr =    arr_options.sample

  current_verb = verbs[cur_con][cur_row][cur_col][cur_arr]

  File.open("documents/text.txt", "a") { |f|
    f.puts @phrase
  }
end

################################################################################################
#                            Selective Decision Tree Revesukyana                               #
################################################################################################
module SpatialRelationships
  class Error < StandardError; end

  class Static_Perimeters

    # The objects within the space.
    def positive_perimeters
      # Base radius of static objects.
      base_radius   = 2500
    
      # Specfic multipliers for Earth index based objects.
      base_two =  2   
      base_fro =  4
      base_six =  6
      base_eit =  8
  
      # Size of specific objects.
      size_of_planets    = base_radius ** base_fro
      size_of_moons      = base_radius ** base_two
      size_of_stars      = base_radius ** base_six
      size_of_blackholes = base_radius ** base_eit
    
      # Total output sizes of specific objects.
      puts "The size of the planets is #{size_of_planets} radius."; sleep(3)
      puts "The size of the moons is #{size_of_moons} radius."; sleep(3)
      puts "The size of the stars is #{size_of_stars} radius."; sleep(3)
      puts "The size of a blackhole is #{size_of_blackholes} radius."; sleep(3)
    end
  
    # Space between the objects.
    def negative_perimeters
      # Base distance between objects.
      base_distance = 1_000_000_000
  
      # Estimated divider between specific objects to base distance.
      space_between_planets    = 43.8
      space_between_moons      = 14.6
      space_between_stars      = 876
      space_between_blackholes = 2628
    
      # Minimum distance between objects.
      planet_distance    = base_distance / space_between_planets
      moon_distance      = base_distance / space_between_moons
      star_distance      = base_distance / space_between_stars
      blackhole_distance = base_distance / space_between_blackholes
    
      # Actual distance between objects
      actual_planets    = planet_distance * 10
      actual_moons      = moon_distance * 10
      actual_stars      = star_distance * 10
      actual_blackholes = blackhole_distance * 10
    
      # The output results of distance between objects.
      puts "The distance between planets is #{actual_planets} miles."; sleep(3)
      puts "The distance between moons is #{actual_moons} miles."; sleep(3)
      puts "The distance between stars is #{actual_stars} miles."; sleep(3)
      puts "The distance between blackholes is #{actual_blackholes} miles."; sleep(3)
    end
  
  end

  # Changing perimeters
  class Dynamic_Perimeters

    # The objects within the space.
    def positive_perimeters
      spaceship     = File.read("data/dynamic/positive_perimenters/spaceship_size.txt").strip.to_i
      space_station = spaceship * 200
      satalite      = space_station / 10
    
      puts "The total size of the space shuttle is #{spaceship} feet."; sleep(3)
      puts "The total size of the space station is #{space_station} feet."; sleep(3)
      puts "The total size of the satalite is #{satalite} feet."; sleep(3)
    end
  
    # Space between the objects.
    def negative_perimeters
      base_multiplier = 10
  
      # Minimum space between objects.
      space_between_spaceships = File.read("data/dynamic/negative_perimeters/space_between_spaceships.txt").strip.to_i
      space_between_station    = File.read("data/dynamic/negative_perimeters/space_between_station.txt").strip.to_i
      space_between_satalite   = File.read("data/dynamic/negative_perimeters/space_between_satalite.txt").strip.to_i
    
      # Actual space between objects
      actual_spaceship_distance = space_between_spaceships * base_multiplier
      actual_station_distance   = space_between_station * base_multiplier
      actual_satalite_distance  = space_between_satalite * base_multiplier
    
      puts "The minimum space between shuttles is #{actual_spaceship_distance} feet."; sleep(3)
      puts "The minimum space between stations is #{actual_station_distance} feet."; sleep(3)
      puts "The minimum space between satalites is #{actual_satalite_distance} feet."; sleep(3)
    end

  end
end

def darly_dreaming
  require "SelfModifiedNaiveBayes"
  require "Selection"

  symbolism = RevisedBayes.new(:elf,             :no_elf),
              RevisedBayes.mew(:fairy,         :no_fairy),
              RevisedBayes.new(:cyclops,     :no_cyclops),
              RevisedBayes.new(:third_eye, :no_third_eye),
              RevisedBayes.new(:clothing,   :no_clothing),
              RevisedBayes.new(:mortality, :no_mortality)
            
  elf_signs     = symbolism[0]
  fairy_signs   = synbikusm[1]
  cyclops_signs = symbolism[2]
  third_eye     = symbolism[3]
  clothing      = symbolism[4]
  mortality     = symbolism[5]

  ## Elf signs
  elf_signs.train(:elf,    "I think she may be an elf.",    "elf")
  elf_signs.train(:no_elf,  "I have never seen an elf.", "no elf")

  ## Fairy signs
  fairy_signs.train(:fairy,    "I think she may be a fairy.",    "fairy")
  fairy_signs.train(:no_fairy,  "I have never seen a fairy.", "no fairy")

  ## Cyclops Signs
  cyclops_signs.train(:elf,    "That person has one giant eye and is ten feet tall.",    "cyclops")
  cyclops_signs.train(:no_elf,     "They seem to be of normal height, and two eyes.", "no cyclops")

  ## Third Eye
  third_eye.train(:third_eye,    "She sees to be able to peer into your thoughts.",    "third eye")
  third_eye.train(:no_third_eye,          "That person cannot predict the future.", "no third eye")

  ## Clothing
  clothing.train(:clothing,    "She appears to be wearing a German dress and wooden shoes.",    "clothing")
  clothing.train(:no_clothing,            "They seem to be of normal height, and two eyes.", "no clothing")

  ## Mortality
  mortality.train(:mortality,    "I had once dreamed that I had died and went back to life.",    "mortality")
  mortality.train(:no_mortality,                          "I dont tend to dream that I die.", "no mortality")

  document = File.readlines("document/dreams/dream_info.txt")
  size_limit = document.size.to_i
  index = 0

  size_limit.times do
    ### Classifications
    elf_class       = elf_signs.classify(document[index])
    fairy_class     = fairy_signs.classify(document[index])
    cyclops_class   = cyclops_signs.classify(document[index])
    third_eye_class = third_eye.classify(document[index])
    clothing_class  = clothing.classify(document[index])
    mortality_class = mortality.classify(document[index])
  
    attribtes = ["Elves"],      ["Fairies"], ["Cyclopses"],
                ["Third Eye"], ["Clothing"], ["Mortality"]
  
    training = [
      [0.00100,      "There seems to be almost no signs of elves in this patient's dream."],
      [0.24825,     "There is beginning to be indicators of elves in this patient's dream"],
      [0.49550, "There seems to be a non neglible signs of elves in this patient's dream."],
      [0.74770,           "There is a very high indicator of elves in this persons dream."],
      [0.99990,         "There are most definitely elves in this patient's current dream."],
    ], [
      [0.00100,      "There seems to be almost no signs of fairies in this patient's dream."],
      [0.24825,     "There is beginning to be indicators of fairies in this patient's dream"],
      [0.49550, "There seems to be a non neglible signs of fairies in this patient's dream."],
      [0.74770,           "There is a very high indicator of fairies in this persons dream."],
      [0.99990,         "There are most definitely fairies in this patient's current dream."],
    ], [
      [0.00100,      "There seems to be almost no signs of cyclopes in this patient's dream."],
      [0.24825,     "There is beginning to be indicators of cyclopes in this patient's dream"],
      [0.49550, "There seems to be a non neglible signs of cyclopes in this patient's dream."],
      [0.74770,           "There is a very high indicator of cyclopes in this persons dream."],
      [0.99990,         "There are most definitely cyclopes in this patient's current dream."],
    ], [
      [0.00100,      "There seems to be almost no signs of third eyes in this patient's dream."],
      [0.24825,     "There is beginning to be indicators of third eyes in this patient's dream"],
      [0.49550, "There seems to be a non neglible signs of third eyes in this patient's dream."],
      [0.74770,           "There is a very high indicator of third eyes in this persons dream."],
      [0.99990,         "There are most definitely third eyes in this patient's current dream."],
    ], [
      [0.00100,      "There seems to be almost no signs of unusual clothing in this patient's dream."],
      [0.24825,     "There is beginning to be indicators of unusual clothing in this patient's dream"],
      [0.49550, "There seems to be a non neglible signs of unusual clothing in this patient's dream."],
      [0.74770,           "There is a very high indicator of unusual clothing in this persons dream."],
      [0.99990,         "There are most definitely unusual clothing in this patient's current dream."],
    ], [
      [0.00100,      "There seems to be almost no signs of sudden mortality in this patient's dream."],
      [0.24825,     "There is beginning to be indicators of sudden mortality in this patient's dream"],
      [0.49550, "There seems to be a non neglible signs of sudden mortality in this patient's dream."],
      [0.74770,           "There is a very high indicator of sudden mortality in this persons dream."],
      [0.99990,         "There are most definitely sudden mortality in this patient's current dream."],
    ]
  
    dec_tree_configurations =    DecisionTree::ID3Tree.new(attribute[0], training[0], 1, :continuous),
                                 DecisionTree::ID3Tree.new(attribute[1], training[1], 1, :continuous),
                                 DecisionTree::ID3Tree.new(attribute[2], training[2], 1, :continuous),
                                 DecisionTree::ID3Tree.new(attribute[3], training[3], 1, :continuous),
                                 DecisionTree::ID3Tree.new(attribute[4], training[4], 1, :continuous),
                                 DecisionTree::ID3Tree.new(attribute[5], training[5], 1, :continuous)

    current_dectree1 = dec_tree_configurations[0]
    current_dectree1.train
  
    current_dectree2 = dec_tree_configurations[1]
    current_dectree2.train

    current_dectree3 = dec_tree_configurations[2]
    current_dectree3.train

    current_dectree4 = dec_tree_configurations[3]
    current_dectree4.train
  
    current_dectree5 = dec_tree_configurations[4]
    current_dectree5.train

    current_dectree6 = dec_tree_configurations[5]
    current_dectree6.train
  
    elf_prediction       = [elf_class[1],                   "There seems to be a non neglible signs of elves in this patient's dream."]
    fairy_prediction     = [fairy_class[1],                "There seems to be a non neglible signs of faires in this patient's dream."]
    cyclops_prediction   = [cyclops_class[1],            "There seems to be a non neglible signs of cyclopes in this patient's dream."]
    third_eye_prediction = [third_eye_class[1],        "There seems to be a non neglible signs of third eyes in this patient's dream."]
    clothing_prediction  = [clothing_class[1],   "There seems to be a non neglible signs of unusual clothing in this patient's dream."]
    mortality_prediction = [mortality_class[1],  "There seems to be a non neglible signs of sudden mortality in this patient's dream."]

    elf_decision       = current_dectree1.predict(elf_prediction)
    fairy_decision     = current_dectree2.predict(fairy_prediction)
    cyclops_decision   = current_dectree3.predict(cyclops_prediction)
    third_eye_decision = current_dectree4.predict(third_eye_prediction)
    clothing_decision  = current_dectree5.predict(clothing_prediction)
    mortality_decision = current_dectree6.predict(mortality_prediction)  

    get_statistics(:elf_decision,         "#{elf_decision}",
                   :fairy_decision,      "#{fairy_decsion}",
                   :cyclops_decision, "#{cyclops_decision}")
  
    dynamic_reward_allocation
  
    get_statistics(:third_eye_decision, "#{third_eye_decision}",
                   :clothing_decision,    "#{clothing_decsion}",
                   :mortality_decision, "#{mortality_decision}")
  
    dynamic_reward_allocation
  
    index = index + 1
  end
end
