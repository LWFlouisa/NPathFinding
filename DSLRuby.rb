require "NeoPathfinding"

a = cette("pomme", "vert")
b = maisette("pomme", "rouge")
c = sinon("pomme", "rouge", "vert")

ouvert("documents/fruit/pomme_vert.txt", "#{a}",
       "documents/fruit/pomme_rouge.txt", "#{b}",
       "documents/fruit/pomme_sinon.txt", "#{c}")
       
fruit_a = File.readlines("documents/fruit/pomme_vert.txt")
fruit_b = File.readlines("documents/fruit/pomme_rouge.txt")
fruit_c = File.readlines("documents/fruit/pomme_sinon.txt")

index_1 = 0
index_2 = 0
index_3 = 0

size_limit = fruit_a.size.to_i

size_limit.times do
  get_statistics(:fruit_a, "#{fruit_a[index_1].strip}",
                 :fruit_b, "#{fruit_b[index_2].strip}",
                 :fruit_c, "#{fruit_c[index_3].strip}")
                 
                 reasses
                 reasses
                 reasses
                 reasses
                 
  index_1 = index_1 + 1
  index_2 = index_2 + 1
  index_3 = index_3 + 1
end
