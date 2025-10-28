package pack;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.DoubleParam;
import java.util.ArrayList;
import java.util.List;

public class CompleteModel2D {

    private Instance _instance;
    private Discretization _discretization;
    private IloCplex _cplex;
    private Box.Orientation _orientation;
    private int _fixedLayer;  // índice de la capa (k)

    private IloNumVar[][] _x;
    private Box[][] _box;

    private boolean _verbose = false;
    private int _solutionCount = 0;
    private double _solveTime = 0.0;

    // NUEVO: guardar las cajas seleccionadas
    private final List<Box> _selected = new ArrayList<>();

    public CompleteModel2D(Instance instance, Discretization discretization,
                           Box.Orientation orientation, int layer) {
        _instance = instance;
        _discretization = discretization;
        _orientation = orientation;
        _fixedLayer = layer;
    }

    public void solve() {
        try {
            _cplex = new IloCplex();
            if (!_verbose)
                _cplex.setOut(null);

            createVariables();
            createObjective();
            createIndependenceConstraints();

            _cplex.setParam(DoubleParam.TimeLimit, 30);

            long start_time = System.nanoTime();
            _cplex.solve();
            long end_time = System.nanoTime();
            _solveTime = (end_time - start_time) / 1_000_000.0;

            if (_cplex.getStatus() == IloCplex.Status.Optimal ||
                _cplex.getStatus() == IloCplex.Status.Feasible) {

                for (int i = 0; i < _discretization.sizeI(); i++)
                for (int j = 0; j < _discretization.sizeJ(); j++)
                    if (_x[i][j] != null && _cplex.getValue(_x[i][j]) > 0.5) {
                        _solutionCount++;
                        _selected.add(_box[i][j]); // guardar la caja seleccionada
                    }
            }

            _cplex.end();
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    protected void createVariables() throws IloException {
        _x = new IloNumVar[_discretization.sizeI()][_discretization.sizeJ()];
        _box = new Box[_discretization.sizeI()][_discretization.sizeJ()];

        int k = _fixedLayer;

        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++) {
            Box b = new Box(i, j, k, _orientation);
            if (b.fits()) {
                _x[i][j] = _cplex.boolVar("x" + i + "" + j + "" + k); // corregido _x
                _box[i][j] = b;
            }
        }
    }

    protected void createObjective() throws IloException {
        IloNumExpr expr = _cplex.linearIntExpr();
        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++)
            if (_x[i][j] != null)
                expr = _cplex.sum(expr, _x[i][j]);
        _cplex.addMaximize(expr);
    }

    protected void createIndependenceConstraints() throws IloException {
        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++) {
            IloNumExpr lhs = _cplex.linearIntExpr();

            for (int ip = 0; ip < _discretization.sizeI(); ip++)
            for (int jp = 0; jp < _discretization.sizeJ(); jp++)
                if (_box[ip][jp] != null && _box[ip][jp].contains(i, j, _fixedLayer))
                    lhs = _cplex.sum(lhs, _x[ip][jp]);

            _cplex.addLe(lhs, 1);
        }
    }

    public int getSolutionCount() {
        return _solutionCount;
    }

    public double getSolveTime() {
        return _solveTime;
    }

    // NUEVO: devolver las cajas seleccionadas
    public List<Box> getSelectedBoxes() {
        return _selected;
    }
}