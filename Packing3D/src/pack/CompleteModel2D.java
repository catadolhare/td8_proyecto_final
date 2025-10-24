package pack;

import ilog.concert.*;
import ilog.cplex.*;
import java.util.ArrayList;
import java.util.List;

public class CompleteModel2D {

    private Instance _instance;
    private Discretization _disc;
    private Box.Orientation _orientation;
    private int _zLayer; // altura base de la capa
    public IloCplex _cplex;
    private IloNumVar[][] _x; // variables de colocación (1 si hay caja en (i,j))
    private double _solveTime;
    private List<int[]> _placedBoxes;

    public CompleteModel2D(Instance instance, Discretization disc, Box.Orientation orientation, int zLayer) {
        _instance = instance;
        _disc = disc;
        _orientation = orientation;
        _zLayer = zLayer;
        _placedBoxes = new ArrayList<>();
    }

    /**
     * Ejecuta el modelo de PLE para una capa 2D y guarda las posiciones usadas.
     */
    public void solve() throws IloException {
        _cplex = new IloCplex();

        int I = _disc.sizeI();
        int J = _disc.sizeJ();
        _x = new IloNumVar[I][J];

        // === Crear variables binarias ===
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                _x[i][j] = _cplex.boolVar("x_" + i + "_" + j);
            }
        }

        // === Parámetros de la caja según la orientación ===
        Box b = new Box(0, 0, _zLayer, _orientation);
        int boxL = b.getLength();
        int boxW = b.getWidth();

        // === Restricciones: no superposición ===
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                if (_disc.getx(i) + boxL > _instance.getL() ||
                    _disc.gety(j) + boxW > _instance.getW()) {
                    // fuera del contenedor
                    _cplex.addEq(_x[i][j], 0);
                }
            }
        }

        // === Restricciones de no solapamiento (PLE clásica 2D) ===
        for (int i1 = 0; i1 < I; i1++) {
            for (int j1 = 0; j1 < J; j1++) {
                for (int i2 = 0; i2 < I; i2++) {
                    for (int j2 = 0; j2 < J; j2++) {
                        if (i1 == i2 && j1 == j2) continue;

                        int x1 = _disc.getx(i1);
                        int y1 = _disc.gety(j1);
                        int x2 = _disc.getx(i2);
                        int y2 = _disc.gety(j2);

                        boolean overlapX = Math.abs(x1 - x2) < boxL;
                        boolean overlapY = Math.abs(y1 - y2) < boxW;

                        if (overlapX && overlapY) {
                            IloLinearNumExpr expr = _cplex.linearNumExpr();
                            expr.addTerm(1, _x[i1][j1]);
                            expr.addTerm(1, _x[i2][j2]);
                            _cplex.addLe(expr, 1, "NoOverlap_" + i1 + "_" + j1 + "_" + i2 + "_" + j2);
                        }
                    }
                }
            }
        }

        // === Función objetivo: maximizar cantidad de cajas ===
        IloLinearNumExpr objective = _cplex.linearNumExpr();
        for (int i = 0; i < I; i++)
            for (int j = 0; j < J; j++)
                objective.addTerm(1, _x[i][j]);

        _cplex.addMaximize(objective);

        // === Resolver ===
        _cplex.setOut(null); // silenciar salida
        long start = System.currentTimeMillis();
        boolean solved = _cplex.solve();
        long end = System.currentTimeMillis();
        _solveTime = end - start;

        if (!solved) {
            System.out.println("⚠️  Modelo 2D no tiene solución factible para orientación " + _orientation);
            return;
        }

        // === Guardar posiciones ocupadas ===
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                if (_cplex.getValue(_x[i][j]) > 0.5) {
                    _placedBoxes.add(new int[]{i, j});
                }
            }
        }
    }

    /** Devuelve cantidad de cajas colocadas en la capa. */
    public int getSolutionCount() {
        return _placedBoxes.size();
    }

    /** Devuelve el tiempo de resolución (ms). */
    public double getSolveTime() {
        return _solveTime;
    }

    /** Devuelve las coordenadas (i,j) de cada caja colocada. */
    public List<int[]> getPlacedBoxes() {
        return _placedBoxes;
    }

    /** Libera memoria de CPLEX. */
    public void end() {
        if (_cplex != null) {
            _cplex.end();
        }
    }
}
