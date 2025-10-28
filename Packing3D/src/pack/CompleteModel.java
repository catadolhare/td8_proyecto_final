package pack;

import java.util.ArrayList;
import java.util.List;

import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.DoubleParam;
import pack.Box.Orientation;
import pack.Callback.Type;

public class CompleteModel {

    private Instance _instance;
    private Discretization _discretization;
    private IloCplex _cplex;
    private ArrayList<Callback> _callbacks;

    private IloNumVar[][][][] _x;
    private Box[][][][] _box;
    private Box.Orientation[] _orientations;

    private boolean _verbose = false;

    // cajas fijas (replicadas desde la fase por capas)
    private List<Box> _fixedBoxes = List.of();

    // === NUEVO: métricas del relleno ===
    private int _packedCount = 0;      // # de cajas colocadas en la fase de relleno
    private double _solveTimeMs = 0.0; // tiempo de resolución en ms

    public CompleteModel(Instance instance, Discretization discretization) {
        _instance = instance;
        _discretization = discretization;
        _orientations = Box.Orientation.values();
        _callbacks = new ArrayList<Callback>();
    }

    // constructor para relleno
    public CompleteModel(Instance instance, Discretization discretization, List<Box> fixedBoxes) {
        this(instance, discretization);
        if (fixedBoxes != null)
            _fixedBoxes = fixedBoxes;
    }

    public void solve() {
        try {
            _cplex = new IloCplex();

            if (_verbose == false)
                _cplex.setOut(null);

            createVariables();
            createObjective();
            createIndependenceConstraints();
            createStabilityConstraints();
            solveModel();

            _cplex.end();
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    protected void createVariables() throws IloException {
        _x = new IloNumVar[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];
        _box = new Box[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];

        for (int i = 0; i < _discretization.sizeI(); ++i)
        for (int j = 0; j < _discretization.sizeJ(); ++j)
        for (int k = 0; k < _discretization.sizeK(); ++k)
        for (int l = 0; l < _orientations.length; ++l) {
            Box box = new Box(i, j, k, _orientations[l]);
            if (!box.fits()) continue;

            // no crear variables que se solapen con fijas
            if (overlapsFixed(box)) continue;

            _x[i][j][k][l] = _cplex.boolVar("x" + i + "" + j + "" + k + "" + _orientations[l]);
            _box[i][j][k][l] = box;
        }

        notify(Type.VariablesCreated);
    }

    // solape con cajas fijas usando contains(...)
    private boolean overlapsFixed(Box b) {
        if (_fixedBoxes == null || _fixedBoxes.isEmpty()) return false;

        for (int i = 0; i < _discretization.sizeI(); ++i)
        for (int j = 0; j < _discretization.sizeJ(); ++j)
        for (int k = 0; k < _discretization.sizeK(); ++k) {
            if (b.contains(i, j, k)) {
                for (Box f : _fixedBoxes) {
                    if (f.contains(i, j, k)) {
                        return true; // comparten al menos una celda → solape
                    }
                }
            }
        }
        return false;
    }

    protected void createObjective() throws IloException {
        IloNumExpr fobj = _cplex.linearIntExpr();

        for (int i = 0; i < _discretization.sizeI(); ++i)
        for (int j = 0; j < _discretization.sizeJ(); ++j)
        for (int k = 0; k < _discretization.sizeK(); ++k)
        for (int l = 0; l < _orientations.length; ++l)
            if (_x[i][j][k][l] != null)
                fobj = _cplex.sum(fobj, _x[i][j][k][l]);

        _cplex.addMaximize(fobj);
        notify(Type.ObjectiveCreated);
    }

    protected void createIndependenceConstraints() throws IloException {
        for (int i = 0; i < _discretization.sizeI(); ++i)
        for (int j = 0; j < _discretization.sizeJ(); ++j)
        for (int k = 0; k < _discretization.sizeK(); ++k) {
            IloNumExpr lhs = _cplex.linearIntExpr();

            for (int ip = 0; ip < _discretization.sizeI(); ++ip)
            for (int jp = 0; jp < _discretization.sizeJ(); ++jp)
            for (int kp = 0; kp < _discretization.sizeK(); ++kp)
            for (int lp = 0; lp < _orientations.length; ++lp)
                if (_box[ip][jp][kp][lp] != null && _box[ip][jp][kp][lp].contains(i, j, k))
                    lhs = _cplex.sum(lhs, _x[ip][jp][kp][lp]);

            _cplex.addLe(lhs, 1, "ind" + i + "" + j + "" + k);
        }
    }

    protected void createStabilityConstraints() throws IloException {
        for (int i = 0; i < _discretization.sizeI(); ++i)
        for (int j = 0; j < _discretization.sizeJ(); ++j)
        for (int k = 1; k < _discretization.sizeK(); ++k)
        for (int l = 0; l < _orientations.length; ++l)
            if (_x[i][j][k][l] != null) {
                IloNumExpr lhs = _cplex.linearIntExpr();
                lhs = _cplex.sum(lhs, _cplex.prod(
                        _instance.getStabilityThreshold() * _box[i][j][k][l].floorSurface(),
                        _x[i][j][k][l]));

                // soporte por variables de decisión
                for (int ip = 0; ip < _discretization.sizeI(); ++ip)
                for (int jp = 0; jp < _discretization.sizeJ(); ++jp)
                for (int kp = 0; kp < _discretization.sizeK(); ++kp)
                for (int lp = 0; lp < _orientations.length; ++lp)
                    if (_box[ip][jp][kp][lp] != null &&
                        _box[ip][jp][kp][lp].getTop() == _box[i][j][k][l].getz())
                        lhs = _cplex.sum(lhs,
                                _cplex.prod(-_box[i][j][k][l].intersectionSurface(_box[ip][jp][kp][lp]),
                                        _x[ip][jp][kp][lp]));

                // soporte de cajas fijas (constante)
                for (Box f : _fixedBoxes)
                    if (f.getTop() == _box[i][j][k][l].getz()) {
                        double inter = _box[i][j][k][l].intersectionSurface(f);
                        if (inter > 0)
                            lhs = _cplex.sum(lhs, -inter);
                    }

                _cplex.addLe(lhs, 0, "stab" + i + "" + j + "" + k + "" + l);
            }

        notify(Type.ConstraintsCreated);
    }

    protected void solveModel() throws IloException {
        _cplex.setParam(DoubleParam.TimeLimit, 60);

        long t0 = System.nanoTime();
        _cplex.solve();
        long t1 = System.nanoTime();
        _solveTimeMs = (t1 - t0) / 1_000_000.0;

        if (_cplex.getStatus() == IloCplex.Status.Optimal || _cplex.getStatus() == IloCplex.Status.Feasible) {
            _packedCount = (int)Math.round(_cplex.getObjValue()); // la FO es suma de x ⇒ #cajas
            System.out.println("Obj value (relleno): " + _packedCount);
        }

        notify(Type.ModelSolved);
    }

    public void register(Callback callback) {
        _callbacks.add(callback);
    }

    public void notify(Callback.Type type) {
        for (Callback c : _callbacks)
            c.notify(type);
    }

    // === NUEVO: getters para métricas del relleno ===
    public int getPackedCount() { return _packedCount; }
    public double getSolveTimeMs() { return _solveTimeMs; }
}