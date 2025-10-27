package pack;

import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import pack.Box.Orientation;

public class CompleteModel2D {
    private Instance _instance;
    private Discretization _discretization;
    private IloCplex _cplex;
    private Orientation _orientation;
    private int _layerIndex;
    private int _solutionCount = 0;
    private double _solveTime = 0;

    public CompleteModel2D(Instance inst, Discretization disc, Orientation orientation, int layerIndex) {
        _instance = inst;
        _discretization = disc;
        _orientation = orientation;
        _layerIndex = layerIndex;
    }

    public void solve() {
        try {
            _cplex = new IloCplex();
            _cplex.setOut(null);

            // variables bin
            int I = _discretization.sizeI();
            int J = _discretization.sizeJ();
            IloNumVar[][] x = new IloNumVar[I][J];

            for (int i = 0; i < I; i++)
                for (int j = 0; j < J; j++)
                    x[i][j] = _cplex.boolVar("x_" + i + "_" + j);

            // objetivo
            IloNumExpr fobj = _cplex.linearNumExpr();
            for (int i = 0; i < I; i++)
                for (int j = 0; j < J; j++)
                    fobj = _cplex.sum(fobj, x[i][j]);
            _cplex.addMaximize(fobj);

            long start = System.nanoTime();
            _cplex.solve();
            long end = System.nanoTime();

            if (_cplex.getStatus() == IloCplex.Status.Optimal || _cplex.getStatus() == IloCplex.Status.Feasible) {
                _solutionCount = (int) Math.round(_cplex.getObjValue());
            }

            _solveTime = (end - start) / 1_000_000.0;
            _cplex.end();

        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    public int getSolutionCount() { return _solutionCount; }
    public double getSolveTime() { return _solveTime; }
}
