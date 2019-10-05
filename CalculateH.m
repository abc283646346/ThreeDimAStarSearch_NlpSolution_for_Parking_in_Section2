function val = CalculateH(node)
global end_index
global three_dim_astar_
val = three_dim_astar_.multiplier_H * (abs(end_index(1) - node(1)) + abs(end_index(2) - node(2))) + randn * randn * 0.01;
end