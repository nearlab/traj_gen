rosservice call /traj_gen/energy_optimal_traj "rOrb: [42164000.0,0.0,0.0]
rStart: [-100.0,0.0,0.0]
vStart: [0.0,0.0,0.0]
rEnd: [100.0,0.0,0.0]
vEnd: [0.0,0.0,0.0]
tStart: 0.0
tEnd: 40000.0
intervals: 40000
grav_param: 398600000000000.0
dist_const: 300.0
sc_thrust: 0.1
time_const: 40000.0
sc_mass: 100.0" >> test.out
