#################################################################################################################
#                                                Main Function                                                  #
#################################################################################################################
# This is the closest python equivalent to Get Statistics, with the main difference being that global variables #
# are assumed to be constants, which means technically I can't reassignment them so far that I've seen. Python  #
# has some other oddities, like the absence of immuteable symbols, which was kind of crucial to the original    #
# implementation.                                                                                               #
#################################################################################################################
def get_statistics(a1, a2, b1, b2, c1, c2):
  import random
  
  a = [ a1, a2 ]
  b = [ b1, b2 ]
  c = [ c1, c2 ]
  
  matrix = [
    [[a[0], a[0], a[0]],
     [a[0], a[0], b[0]],
     [a[0], a[0], b[0]]],
     
    [[b[0], b[0], a[0]],
     [b[0], b[0], b[0]],
     [b[0], b[0], b[0]]],
     
    [[c[0], c[0], a[0]],
     [c[0], c[0], b[0]],
     [c[0], c[0], c[0]]],
  ], [
    [[a[1], a[1], a[1]],
     [a[1], a[1], b[1]],
     [a[1], a[1], b[1]]],
     
    [[b[1], b[1], a[1]],
     [b[1], b[1], b[1]],
     [b[1], b[1], b[1]]],
     
    [[c[1], c[1], a[1]],
     [c[1], c[1], b[1]],
     [c[1], c[1], c[1]]],
  ], [
    [[0.33, 0.33, 0.33],
     [0.33, 0.33, 0.33],
     [0.33, 0.33, 0.33]],
     
    [[0.33, 0.33, 0.33],
     [0.33, 0.33, 0.33],
     [0.33, 0.33, 0.33]],
     
    [[0.33, 0.33, 0.33],
     [0.33, 0.33, 0.33],
     [0.33, 0.33, 0.33]],
  ]
  
  labels           =  matrix[0]
  definition       =  matrix[1]
  base_probability = matrix[2]
 
  selection_probability = 0.33 * 0.33
  
  row_options = [0, 1, 2]
  col_options = [0, 1, 2]
  arr_options = [0, 1, 2]
  
  cur_row = random.choice(row_options)
  cur_col = random.choice(col_options)
  cur_arr = random.choice(arr_options)
  
  #global current_label
  #global current_description
  #global current_probability
  
  current_label       =      labels[cur_row][cur_col][cur_arr]
  current_description =  definition[cur_row][cur_col][cur_arr]
  
  index_probability = base_probability[cur_row][cur_col][cur_arr]
  current_probability = index_probability * selection_probability

  global my_data

  my_data = current_label, current_description, current_probability
  
#################################################################################################################
#                                        Granular Confidence Reassignment                                       #
#################################################################################################################
# The weird thing about Python is that global variables are assumed to be constants, which means you cant       #
# easily reassign them. This approach works around the problem by creating level of reassessment that only work #
# if a reassessment is already done.                                                                            #
#                                                                                                               #
# In effect this means that you can constantly reassign to a global variable using the same function, which     #
# kind of defeats the purpose of making the variable globally accessible.                                       #
#################################################################################################################
def reasses_l1():
  current_probability = my_data[2] + my_data[2]

  global my_data2
  
  my_data2 = my_data[0], my_data[1], current_probability
  
  print(my_data2)
  
def reasses_l2():
  current_probability = my_data2[2] + my_data2[2]

  global my_data3
  
  my_data3 = my_data[0], my_data[1], current_probability
  
  print(my_data3)
  
def reasses_l3():
  current_probability = my_data3[2] + my_data3[2]

  global my_data4
  
  my_data4 = my_data[0], my_data[1], current_probability
  
  print(my_data4)
  
def reasses_l4():
  current_probability = my_data4[2] + my_data4[2]

  global my_data5
  
  my_data5 = my_data[0], my_data[1], current_probability
  
  print(my_data5)
  

## Levels of reconsideration
def reconsider_l1():
  current_probability = my_data[2] + my_data[2]

  global my_data2
  
  my_data2 = my_data[0], my_data[1], current_probability
  
  print(my_data2)
  
def reconsider_l2():
  current_probability = my_data2[2] + my_data2[2]

  global my_data3
  
  my_data3 = my_data[0], my_data[1], current_probability
  
  print(my_data3)
  
def reconsider_l3():
  current_probability = my_data3[2] + my_data3[2]

  global my_data4
  
  my_data4 = my_data[0], my_data[1], current_probability
  
  print(my_data4)
  
def reconsider_l4():
  current_probability = my_data4[2] + my_data4[2]

  global my_data5
  
  my_data5 = my_data[0], my_data[1], current_probability
  
  print(my_data5)

get_statistics(":fairy_symbolism", "There is fairy symbolism.",
               ":snake_symbolism", "There is snake symbolism.",
               ":orc_symbolism",     "There is orc symbolism.")

print(my_data)

reasses_l1()
reasses_l2()
reasses_l3()
reasses_l4()

#reconsider_l4()
#reconsider_l3()
#reconsider_l2()
#reconsider_l1()
