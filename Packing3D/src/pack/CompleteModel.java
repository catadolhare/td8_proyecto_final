package pack;

import java.util.ArrayList;
import ilog.concert.*;
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

    // máscara de celdas ocupadas por bloques (proveniente del BlockModel)
    private boolean[][][] _occupiedByBlocks;

    private String status = "Unknown";
    public String getStatus() { return status; }

    private double _objValue = 0.0; // # de cajas NUEVAS en huecos
    public double getObjValue() { return _objValue; }

    public CompleteModel(Instance instance, Discretization discretization) {
        _instance = instance;
        _discretization = discretization;
        _orientations = Box.Orientation.values();
        _callbacks = new ArrayList<Callback>();
        _occupiedByBlocks = null; // se setea desde fuera
    }

    // Setear máscara de celdas ocupadas por bloques
    public void setOccupiedMask(boolean[][][] occupiedMask) {
        _occupiedByBlocks = occupiedMask;
    }

    // compatibilidad con versiones previas: banear una celda puntual
    public void banCell(int i, int j, int k) {
        if (_occupiedByBlocks == null) {
            _occupiedByBlocks = new boolean[_discretization.sizeI()][_discretization.sizeJ()][_discretization.sizeK()];
        }
        if (i>=0 && i<_discretization.sizeI() &&
            j>=0 && j<_discretization.sizeJ() &&
            k>=0 && k<_discretization.sizeK()) {
            _occupiedByBlocks[i][j][k] = true;
        }
    }

    public void addBannedPosition(int i, int j, int k, Orientation o) { banCell(i,j,k); }

    public void solve() {
        try {
            _cplex = new IloCplex();
            if (!_verbose) _cplex.setOut(null);

            createVariables();               // evita crear variables que solapen celdas ocupadas
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

        for (int i=0; i<_discretization.sizeI(); ++i)
        for (int j=0; j<_discretization.sizeJ(); ++j)
        for (int k=0; k<_discretization.sizeK(); ++k)
        for (int l=0; l<_orientations.length; ++l) {

            Box box = new Box(i, j, k, _orientations[l]);
            if (!box.fits()) continue;

            // DESCARTAR si el volumen de la caja solapa con alguna celda ocupada por bloques
            if (_occupiedByBlocks != null && overlapsOccupiedCells(box)) {
                continue;
            }

            _x[i][j][k][l] = _cplex.boolVar("x" + i + "_" + j + "_" + k + "_" + _orientations[l]);
            _box[i][j][k][l] = box;
        }

        notify(Type.VariablesCreated);
    }

    // Chequeo de solape optimizado (usa bounding box físico del candidato)
    private boolean overlapsOccupiedCells(Box c) {
        if (_occupiedByBlocks == null) return false;

        int I = _discretization.sizeI(), J = _discretization.sizeJ(), K = _discretization.sizeK();

        int x0 = _discretization.getx(c.geti());
        int y0 = _discretization.gety(c.getj());
        int z0 = _discretization.getz(c.getk());
        int ex = x0 + c.getLength();
        int ey = y0 + c.getWidth();
        int ez = z0 + c.getHeight();

        for (int ii = 0; ii < I; ii++)
        for (int jj = 0; jj < J; jj++)
        for (int kk = 0; kk < K; kk++) {
            if (!_occupiedByBlocks[ii][jj][kk]) continue;
            int px = _discretization.getx(ii);
            int py = _discretization.gety(jj);
            int pz = _discretization.getz(kk);
            if (px >= x0 && px < ex && py >= y0 && py < ey && pz >= z0 && pz < ez) {
                return true;
            }
        }
        return false;
    }

    protected void createObjective() throws IloException {
        IloNumExpr fobj = _cplex.linearIntExpr();

        for (int i=0; i<_discretization.sizeI(); ++i)
        for (int j=0; j<_discretization.sizeJ(); ++j)
        for (int k=0; k<_discretization.sizeK(); ++k)
        for (int l=0; l<_orientations.length; ++l) {
            if (_x[i][j][k][l] != null)
                fobj = _cplex.sum(fobj, _x[i][j][k][l]);
        }

        _cplex.addMaximize(fobj);
        notify(Type.ObjectiveCreated);
    }

    protected void createIndependenceConstraints() throws IloException {
        for (int i=0; i<_discretization.sizeI(); ++i)
        for (int j=0; j<_discretization.sizeJ(); ++j)
        for (int k=0; k<_discretization.sizeK(); ++k) {
            IloNumExpr lhs = _cplex.linearIntExpr();

            for (int ip=0; ip<_discretization.sizeI(); ++ip)
            for (int jp=0; jp<_discretization.sizeJ(); ++jp)
            for (int kp=0; kp<_discretization.sizeK(); ++kp)
            for (int lp=0; lp<_orientations.length; ++lp) {
                if (_box[ip][jp][kp][lp] != null && _box[ip][jp][kp][lp].contains(i,j,k))
                    lhs = _cplex.sum(lhs, _x[ip][jp][kp][lp]);
            }

            _cplex.addLe(lhs, 1, "ind" + i + "_" + j + "_" + k);
        }
    }

    protected void createStabilityConstraints() throws IloException {
        for (int i=0; i<_discretization.sizeI(); ++i)
        for (int j=0; j<_discretization.sizeJ(); ++j)
        for (int k=1; k<_discretization.sizeK(); ++k)
        for (int l=0; l<_orientations.length; ++l) if (_x[i][j][k][l] != null) {
            IloNumExpr lhs = _cplex.linearIntExpr();
            lhs = _cplex.sum(lhs, _cplex.prod(_instance.getStabilityThreshold() * _box[i][j][k][l].floorSurface(), _x[i][j][k][l]));

            for (int ip=0; ip<_discretization.sizeI(); ++ip)
            for (int jp=0; jp<_discretization.sizeJ(); ++jp)
            for (int kp=0; kp<_discretization.sizeK(); ++kp)
            for (int lp=0; lp<_orientations.length; ++lp)
                if (_box[ip][jp][kp][lp] != null && _box[ip][jp][kp][lp].getTop() == _box[i][j][k][l].getz()) {
                    lhs = _cplex.sum(lhs, _cplex.prod(-_box[i][j][k][l].intersectionSurface(_box[ip][jp][kp][lp]), _x[ip][jp][kp][lp]));
                }

            _cplex.addLe(lhs, 0, "stab" + i + "_" + j + "_" + k + "_" + l);
        }

        notify(Type.ConstraintsCreated);
    }

    protected void solveModel() throws IloException {
        _cplex.setParam(DoubleParam.TimeLimit, 600);
        _cplex.solve();
        status = _cplex.getStatus().toString();

        System.out.println("Status (relleno): " + _cplex.getStatus());
        if (_cplex.getStatus() == IloCplex.Status.Optimal || _cplex.getStatus() == IloCplex.Status.Feasible) {
            _objValue = _cplex.getObjValue();   // cajas NUEVAS en huecos
            System.out.println("Obj (relleno) = " + _objValue);
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
}