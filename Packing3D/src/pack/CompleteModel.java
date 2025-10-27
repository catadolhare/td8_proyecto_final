package pack;

import java.util.ArrayList;

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
    public IloCplex _cplex;  // público solo para lectura desde LayerHeuristic
    private ArrayList<Callback> _callbacks;

    private IloNumVar[][][][] _x;
    private Box[][][][] _box;
    private Box.Orientation[] _orientations;

    private boolean _verbose = false;

    public CompleteModel(Instance instance, Discretization discretization) {
        _instance = instance;
        _discretization = discretization;
        _orientations = Box.Orientation.values();
        _callbacks = new ArrayList<Callback>();
    }

    // === SOLUCIÓN PRINCIPAL ===
    public void solve() {
        try {
            // ====== PRIMERA FASE: RELAJACIÓN LINEAL ======
            _cplex = new IloCplex();
            if (!_verbose) _cplex.setOut(null);

            createVariablesRelaxed();
            createObjective();
            createIndependenceConstraints();
            createStabilityConstraints();
            _cplex.solve();

            double eps = 1e-4;
            int eliminadas = 0;

            for (int i = 0; i < _discretization.sizeI(); i++)
            for (int j = 0; j < _discretization.sizeJ(); j++)
            for (int k = 0; k < _discretization.sizeK(); k++)
            for (int l = 0; l < _orientations.length; l++) {
                if (_x[i][j][k][l] != null && _cplex.getValue(_x[i][j][k][l]) < eps) {
                    removeVariable(i, j, k, _orientations[l]);
                    eliminadas++;
                }
            }
            System.out.println("Cantidad de variables eliminadas tras la relajación lineal: " + eliminadas);

            // ====== SEGUNDA FASE: MODELO ENTERO REDUCIDO ======
            _cplex.end();
            _cplex = new IloCplex();
            if (!_verbose) _cplex.setOut(null);

            createVariables();
            createObjective();
            createIndependenceConstraints();
            createStabilityConstraints();

            long start = System.nanoTime();
            solveModel();
            long end = System.nanoTime();

            double tiempoTotal = (end - start) / 1_000_000.0;
            System.out.println("Tiempo de resolución (segunda pasada) = " + (tiempoTotal / 1000.0) + " segundos");

            _cplex.end();
        } catch (IloException e) {
            e.printStackTrace();
        }
    }

    // === CREACIÓN DE VARIABLES ===
    protected void createVariables() throws IloException {
        _x = new IloNumVar[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];
        _box = new Box[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];

        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++)
        for (int k = 0; k < _discretization.sizeK(); k++)
        for (int l = 0; l < _orientations.length; l++) {
            Box b = new Box(i, j, k, _orientations[l]);
            if (b.fits()) {
                _x[i][j][k][l] = _cplex.boolVar("x" + i + "_" + j + "_" + k + "_" + _orientations[l]);
                _box[i][j][k][l] = b;
            }
        }
        notify(Type.VariablesCreated);
    }

    protected void createVariablesRelaxed() throws IloException {
        _x = new IloNumVar[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];
        _box = new Box[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];

        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++)
        for (int k = 0; k < _discretization.sizeK(); k++)
        for (int l = 0; l < _orientations.length; l++) {
            Box b = new Box(i, j, k, _orientations[l]);
            if (b.fits()) {
                _x[i][j][k][l] = _cplex.numVar(0.0, 1.0, "x" + i + "_" + j + "_" + k + "_" + _orientations[l]);
                _box[i][j][k][l] = b;
            }
        }
        notify(Type.VariablesCreated);
    }

    // === OBJETIVO ===
    protected void createObjective() throws IloException {
        IloNumExpr fobj = _cplex.linearNumExpr();
        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++)
        for (int k = 0; k < _discretization.sizeK(); k++)
        for (int l = 0; l < _orientations.length; l++) {
            if (_x[i][j][k][l] != null) fobj = _cplex.sum(fobj, _x[i][j][k][l]);
        }
        _cplex.addMaximize(fobj);
        notify(Type.ObjectiveCreated);
    }

    // === RESTRICCIONES ===
    protected void createIndependenceConstraints() throws IloException {
        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++)
        for (int k = 0; k < _discretization.sizeK(); k++) {
            IloNumExpr lhs = _cplex.linearNumExpr();

            for (int ip = 0; ip < _discretization.sizeI(); ip++)
            for (int jp = 0; jp < _discretization.sizeJ(); jp++)
            for (int kp = 0; kp < _discretization.sizeK(); kp++)
            for (int lp = 0; lp < _orientations.length; lp++) {
                if (_box[ip][jp][kp][lp] != null && _box[ip][jp][kp][lp].contains(i, j, k))
                    lhs = _cplex.sum(lhs, _x[ip][jp][kp][lp]);
            }
            _cplex.addLe(lhs, 1, "ind_" + i + "_" + j + "_" + k);
        }
    }

    protected void createStabilityConstraints() throws IloException {
        for (int i = 0; i < _discretization.sizeI(); i++)
        for (int j = 0; j < _discretization.sizeJ(); j++)
        for (int k = 1; k < _discretization.sizeK(); k++)
        for (int l = 0; l < _orientations.length; l++) {
            if (_x[i][j][k][l] == null) continue;

            IloNumExpr lhs = _cplex.linearNumExpr();
            lhs = _cplex.sum(lhs, _cplex.prod(_instance.getStabilityThreshold() * _box[i][j][k][l].floorSurface(), _x[i][j][k][l]));

            for (int ip = 0; ip < _discretization.sizeI(); ip++)
            for (int jp = 0; jp < _discretization.sizeJ(); jp++)
            for (int kp = 0; kp < _discretization.sizeK(); kp++)
            for (int lp = 0; lp < _orientations.length; lp++) {
                if (_box[ip][jp][kp][lp] != null && _box[ip][jp][kp][lp].getTop() == _box[i][j][k][l].getz()) {
                    lhs = _cplex.sum(lhs,
                            _cplex.prod(-_box[i][j][k][l].intersectionSurface(_box[ip][jp][kp][lp]), _x[ip][jp][kp][lp]));
                }
            }
            _cplex.addLe(lhs, 0, "stab_" + i + "_" + j + "_" + k + "_" + l);
        }
        notify(Type.ConstraintsCreated);
    }

    // === SOLUCIÓN FINAL ===
    protected void solveModel() throws IloException {
        _cplex.setParam(DoubleParam.TimeLimit, 600);
        _cplex.solve();

        System.out.println(_cplex.getStatus());
        if (_cplex.getStatus() == IloCplex.Status.Optimal || _cplex.getStatus() == IloCplex.Status.Feasible) {
            System.out.println("Obj value = " + _cplex.getObjValue());
        }
        notify(Type.ModelSolved);
    }

    // === AUX ===
    public void register(Callback cb) { _callbacks.add(cb); }

    public void notify(Callback.Type t) {
        for (Callback c : _callbacks) c.notify(t);
    }

    public void removeVariable(int i, int j, int k, Orientation o) {
        for (int lp = 0; lp < _orientations.length; lp++)
            if (_orientations[lp] == o) {
                _x[i][j][k][lp] = null;
                _box[i][j][k][lp] = null;
            }
    }
}
