function c = map_const()
%MAP_CONST  Shared occupancy-grid constants for Day4_5 from-scratch rewrite.
%
%   c = map_const() returns a struct of compile-time constants used by every
%   module in dev/day4_5 (add_obstacle, hybrid_astar_plan, etc.). Centralising
%   them here makes the cell-count / resolution change (plan item 1) a single
%   edit instead of a hunt across files.
%
%   Plan item 1 sizing:
%       Physical area : x in [X_MIN, X_MAX] = [0, 100] m
%                       y in [Y_MIN, Y_MAX] = [-100, 0] m  (100 m x 100 m)
%       Resolution    : 0.5 m / cell
%       Grid          : 200 rows x 200 cols
%
%   Grid convention (matches legacy chart_45):
%       wx(col) = X_MIN + (col - 0.5) * RES   ->  col = floor((wx - X_MIN)/RES) + 1
%       wy(row) = Y_MAX - (row - 0.5) * RES   ->  row = floor((Y_MAX - wy)/RES) + 1
%
%#codegen

c.N        = int32(200);
c.RES      = 0.5;
c.X_MIN    = 0.0;
c.X_MAX    = 100.0;
c.Y_MIN    = -100.0;
c.Y_MAX    = 0.0;

% Volvo FH + Silo trailer (T01..T07), from TestRun day4_5_scenario1
c.TRUCK_W  = 2.48;
c.TRUCK_L  = 11.5;

% Ego vehicle (Kia EV6 approx)
c.EGO_W    = 1.9;
c.EGO_L    = 4.7;
c.WHEELBASE = 2.8;

% Safety margin added to obstacle half-width / half-length
c.SAFETY_MARGIN = 0.3;

end
