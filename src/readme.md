
# Glossary
V = vertex set
A = arc set
q = load (q_i)
d = service duration (d_i)
[e_i, l_i] = time window (non-negative)
T = end of planning horizon
Q_k = load limit fovehicle k
L = maximum ride time
A_i = arrival time at vertex i v_i
B_i = beginning of service at vertex v_i
         B_i >= max(e_i, A_i)
D_i = departure time from vertex v_i
         B_i + d_i
W_i = waiting time at vertex v_i
         W_i = B_i - A_i
L_i = ride time for request i
         L_i = B_(i+n) - D_i
 F_i = forward time slack of vertex v_i
 P_j = ride time of user who's destination is vertex v_j
