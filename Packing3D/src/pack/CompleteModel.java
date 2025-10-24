package pack;

import ilog.concert.*;
import ilog.cplex.*;

public class CompleteModel {

    private Instance _instance;
    private Discretization _disc;
    public IloCplex _cplex;
    private IloNumVar[][][][] _x; // x[i][j][k][o] = 1 si hay caja en (i,j,k) con orientación o
    private double _solveTime;
    private boolean[][][] _occupiedInput;

    public CompleteModel(Instance instance, Discretization disc, boolean[][][] occupied) {
        _instance = instance;
        _disc = disc;
        _occupiedInput = occupied;
    }

    public void solve() throws IloException {
        _cplex = new IloCplex();
        int I = _disc.sizeI();
        int J = _disc.sizeJ();
        int K = _disc.sizeK();
        Box.Orientation[] orientations = Box.Orientation.values();

        // === Variables ===
        _x = new IloNumVar[I][J][K][orientations.length];
        for (int i = 0; i < I; i++)
            for (int j = 0; j < J; j++)
                for (int k = 0; k < K; k++)
                    for (int o = 0; o < orientations.length; o++)
                        _x[i][j][k][o] = _cplex.boolVar("x_" + i + "_" + j + "_" + k + "_" + orientations[o]);

        // === Restricciones: una sola orientación por celda ===
        for (int i = 0; i < I; i++)
            for (int j = 0; j < J; j++)
                for (int k = 0; k < K; k++) {
                    IloLinearNumExpr expr = _cplex.linearNumExpr();
                    for (int o = 0; o < orientations.length; o++)
                        expr.addTerm(1, _x[i][j][k][o]);
                    _cplex.addLe(expr, 1, "OneOrient_" + i + "_" + j + "_" + k);
                }

        // === Restricciones: no colocar cajas fuera del contenedor o en celdas ocupadas ===
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                for (int k = 0; k < K; k++) {
                    for (int o = 0; o < orientations.length; o++) {
                        Box b = new Box(i, j, k, orientations[o]);

                        // Si la caja no cabe o está fuera, o la celda ya está ocupada, la variable se fija en 0
                        if (!b.fits() || (_occupiedInput != null && _occupiedInput[i][j][k])) {
                            _cplex.addEq(_x[i][j][k][o], 0);
                        }
                    }
                }
            }
        }

        // === Restricciones de no solapamiento ===
        for (int i1 = 0; i1 < I; i1++) {
            for (int j1 = 0; j1 < J; j1++) {
                for (int k1 = 0; k1 < K; k1++) {
                    for (int o1 = 0; o1 < orientations.length; o1++) {
                        Box b1 = new Box(i1, j1, k1, orientations[o1]);
                        if (!b1.fits()) continue;

                        for (int i2 = 0; i2 < I; i2++) {
                            for (int j2 = 0; j2 < J; j2++) {
                                for (int k2 = 0; k2 < K; k2++) {
                                    for (int o2 = 0; o2 < orientations.length; o2++) {
                                        if (i1 == i2 && j1 == j2 && k1 == k2 && o1 == o2) continue;
                                        Box b2 = new Box(i2, j2, k2, orientations[o2]);
                                        if (!b2.fits()) continue;

                                        boolean overlapX = !(b1.getx() + b1.getLength() <= b2.getx() || b2.getx() + b2.getLength() <= b1.getx());
                                        boolean overlapY = !(b1.gety() + b1.getWidth() <= b2.gety() || b2.gety() + b2.getWidth() <= b1.gety());
                                        boolean overlapZ = !(b1.getz() + b1.getHeight() <= b2.getz() || b2.getz() + b2.getHeight() <= b1.getz());

                                        if (overlapX && overlapY && overlapZ) {
                                            IloLinearNumExpr expr = _cplex.linearNumExpr();
                                            expr.addTerm(1, _x[i1][j1][k1][o1]);
                                            expr.addTerm(1, _x[i2][j2][k2][o2]);
                                            _cplex.addLe(expr, 1);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // === Función objetivo: maximizar cajas colocadas ===
        IloLinearNumExpr obj = _cplex.linearNumExpr();
        for (int i = 0; i < I; i++)
            for (int j = 0; j < J; j++)
                for (int k = 0; k < K; k++)
                    for (int o = 0; o < orientations.length; o++)
                        obj.addTerm(1, _x[i][j][k][o]);
        _cplex.addMaximize(obj);

        // === Resolver ===
        _cplex.setOut(null);
        long start = System.currentTimeMillis();
        boolean solved = _cplex.solve();
        long end = System.currentTimeMillis();
        _solveTime = end - start;

        if (!solved) {
            System.out.println("⚠️  Modelo 3D (multi-orientación) no tiene solución factible para el relleno.");
            return;
        }

        System.out.println("Estado CPLEX (relleno): " + _cplex.getStatus());
        System.out.println("Valor objetivo CPLEX (relleno): " + _cplex.getObjValue());
    }

    public double getSolveTime() {
        return _solveTime;
    }

    public void end() {
        if (_cplex != null)
            _cplex.end();
    }
}
