package pack;

import java.util.ArrayList;
import ilog.concert.*;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.DoubleParam;
import pack.Box.Orientation;
import pack.Callback.Type;

public class CompleteModel {
    private Instance _instance;
    protected Discretization _discretization;
    protected IloCplex _cplex;
    private ArrayList<Callback> _callbacks;

    protected IloNumVar[][][][] _x;
    protected Box[][][][] _box;
    private Box.Orientation[] _orientations;

    private boolean _verbose = false;
    private boolean[][][] _occupied; // matriz que indica posiciones ocupadas (true = ya ocupada)

    private double _objectiveValue = 0.0;
    private ilog.cplex.IloCplex.Status _status = null;


    // === Constructor clásico ===
    public CompleteModel(Instance instance, Discretization discretization) {
        _instance = instance;
        _discretization = discretization;
        _orientations = Box.Orientation.values();
        _callbacks = new ArrayList<>();
        _occupied = null;
    }

    // === Constructor extendido con matriz ocupada ===
    public CompleteModel(Instance instance, Discretization discretization, boolean[][][] occupied) {
        this(instance, discretization);
        this._occupied = occupied;
    }

    // === Método principal de resolución ===
    public void solve() {
        try {
            _cplex = new IloCplex();
            if (!_verbose) _cplex.setOut(null);

            createVariables();
            createObjective();
            createIndependenceConstraints();
            createStabilityConstraints();

            _cplex.setParam(DoubleParam.TimeLimit, 180); // 3 minutos
            _cplex.solve();

            System.out.println(_cplex.getStatus());
            if (_cplex.getStatus() == IloCplex.Status.Optimal || _cplex.getStatus() == IloCplex.Status.Feasible) {
                for (int i = 0; i < _discretization.sizeI(); ++i)
                    for (int j = 0; j < _discretization.sizeJ(); ++j)
                        for (int k = 0; k < _discretization.sizeK(); ++k)
                            for (int l = 0; l < _orientations.length; ++l)
                                if (_x[i][j][k][l] != null && _cplex.getValue(_x[i][j][k][l]) > 0.1) {
                                    System.out.printf("x[%d,%d,%d,%s] = %.1f (x=%d, y=%d, z=%d)%n",
                                            i, j, k, _orientations[l], _cplex.getValue(_x[i][j][k][l]),
                                            _discretization.getx(i), _discretization.gety(j), _discretization.getz(k));
                                }
                _status = _cplex.getStatus();

                if (_status == IloCplex.Status.Optimal || _status == IloCplex.Status.Feasible) {
                    try {
                        _objectiveValue = _cplex.getObjValue();
                    } catch (IloException e) {
                        _objectiveValue = 0.0;
                    }
                }


                System.out.println("Obj value = " + _cplex.getObjValue());
            }

            notify(Type.ModelSolved);
            _cplex.end();

        } catch (IloException e) {
            e.printStackTrace();
        }
    }
    
    public double getObjectiveValue() {
        return _objectiveValue;
    }

    public ilog.cplex.IloCplex.Status getStatus() {
        return _status;
    }


    // === Creación de variables ===
    protected void createVariables() throws IloException {
        _x = new IloNumVar[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];
        _box = new Box[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()][_orientations.length];

        for (int i = 0; i < _discretization.sizeI(); ++i)
            for (int j = 0; j < _discretization.sizeJ(); ++j)
                for (int k = 0; k < _discretization.sizeK(); ++k)
                    for (int l = 0; l < _orientations.length; ++l) {

                        // Si la celda ya está ocupada, se omite
                        if (_occupied != null && _occupied[i][j][k])
                            continue;

                        Box box = new Box(i, j, k, _orientations[l]);
                        if (box.fits()) {
                            _x[i][j][k][l] = _cplex.boolVar("x_" + i + "_" + j + "_" + k + "_" + _orientations[l]);
                            _box[i][j][k][l] = box;
                        }
                    }

        notify(Type.VariablesCreated);
    }

    // === Función objetivo: maximizar número de cajas ===
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

    // === Restricciones de independencia (no solapamiento) ===
    protected void createIndependenceConstraints() throws IloException {
        for (int i = 0; i < _discretization.sizeI(); ++i)
            for (int j = 0; j < _discretization.sizeJ(); ++j)
                for (int k = 0; k < _discretization.sizeK(); ++k) {

                    // Si la celda está ocupada (por las capas), la fijamos como ya llena
                    if (_occupied != null && _occupied[i][j][k])
                        continue;

                    IloNumExpr lhs = _cplex.linearIntExpr();

                    for (int ip = 0; ip < _discretization.sizeI(); ++ip)
                        for (int jp = 0; jp < _discretization.sizeJ(); ++jp)
                            for (int kp = 0; kp < _discretization.sizeK(); ++kp)
                                for (int lp = 0; lp < _orientations.length; ++lp)
                                    if (_box[ip][jp][kp][lp] != null && _box[ip][jp][kp][lp].contains(i, j, k))
                                        lhs = _cplex.sum(lhs, _x[ip][jp][kp][lp]);

                    _cplex.addLe(lhs, 1, "ind_" + i + "_" + j + "_" + k);
                }
    }

    // === Restricciones de estabilidad (no flotar) ===
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

                            for (int ip = 0; ip < _discretization.sizeI(); ++ip)
                                for (int jp = 0; jp < _discretization.sizeJ(); ++jp)
                                    for (int kp = 0; kp < _discretization.sizeK(); ++kp)
                                        for (int lp = 0; lp < _orientations.length; ++lp)
                                            if (_box[ip][jp][kp][lp] != null
                                                    && _box[ip][jp][kp][lp].getTop() == _box[i][j][k][l].getz()) {
                                                lhs = _cplex.sum(lhs,
                                                        _cplex.prod(-_box[i][j][k][l]
                                                                .intersectionSurface(_box[ip][jp][kp][lp]),
                                                                _x[ip][jp][kp][lp]));
                                            }

                            _cplex.addLe(lhs, 0, "stab_" + i + "_" + j + "_" + k + "_" + l);
                        }

        notify(Type.ConstraintsCreated);
    }

    // === Gestión de callbacks (por si se usan en CPLEX) ===
    public void register(Callback callback) {
        _callbacks.add(callback);
    }

    public void notify(Callback.Type type) {
        for (Callback c : _callbacks)
            c.notify(type);
    }

    // === Eliminar una variable específica ===
    public void removeVariable(int i, int j, int k, Orientation l) {
        for (int lp = 0; lp < _orientations.length; ++lp)
            if (_orientations[lp] == l) {
                _x[i][j][k][lp] = null;
                _box[i][j][k][lp] = null;
            }
    }
}
