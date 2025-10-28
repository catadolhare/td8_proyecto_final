package pack;

import java.util.ArrayList;
import java.util.List;

public class LayerHeuristic {

    // === NUEVO: almacenar últimos resultados ===
    private static int _lastBaseBoxes = 0;
    private static int _lastResidualBoxes = 0;
    private static int _lastTotalBoxes = 0;
    private static double _lastTotalTimeMs = 0.0;

    public static void run(Instance instance) {
        Discretization disc = new Discretization(instance);

        int bestTotal = 0;
        Box.Orientation bestOrientation = null;
        int bestBoxesPerLayer = 0;
        int bestLayers = 0;
        double bestTime = 0.0; // ms de resolver la capa (1 solve)

        System.out.println("=== Heurística por Capas (versión simplificada) ===");
        System.out.println("Contenedor: L=" + instance.getL() + " W=" + instance.getW() + " H=" + instance.getH());
        System.out.println();

        for (Box.Orientation o : Box.Orientation.values()) {
            Box sample = new Box(0, 0, 0, o);
            int layerHeight = sample.getHeight();
            int layers = instance.getH() / layerHeight;

            CompleteModel2D model = new CompleteModel2D(instance, disc, o, 0);
            model.solve();

            int boxesInLayer = model.getSolutionCount();
            double solveTime = model.getSolveTime(); // ms

            int totalBoxes = boxesInLayer * layers;

            System.out.printf(
                "Orientación %-10s → %3d cajas por capa × %2d capas = %3d cajas totales\n",
                o.toString(), boxesInLayer, layers, totalBoxes
            );

            if (totalBoxes > bestTotal) {
                bestTotal = totalBoxes;
                bestOrientation = o;
                bestBoxesPerLayer = boxesInLayer;
                bestLayers = layers;
                bestTime = solveTime; // guardamos el tiempo de la mejor orientación
            }
        }

        System.out.println("\n=== Resultado Final ===");
        System.out.println("Mejor orientación: " + bestOrientation);
        System.out.println("Cajas por capa: " + bestBoxesPerLayer);
        System.out.println("Cantidad de capas: " + bestLayers);
        System.out.println("Total de cajas en el contenedor: " + bestTotal);
        System.out.printf("Tiempo total de resolución (1 capa): %.2f segundos\n", bestTime / 1000.0);

        // === FASE DE RELLENO 3D ===
        System.out.println("\n=== Fase de Relleno (espacios vacíos) ===");
        CompleteModel2D bestModel = new CompleteModel2D(instance, disc, bestOrientation, 0);
        bestModel.solve();
        List<Box> baseLayer = bestModel.getSelectedBoxes();

        List<Box> fixed = new ArrayList<>();
        for (int zLayer = 0; zLayer < bestLayers; zLayer++) {
            for (Box b0 : baseLayer) {
                Box b = new Box(b0.geti(), b0.getj(), b0.getk() + zLayer, b0.getOrientation());
                fixed.add(b);
            }
        }

        CompleteModel residual = new CompleteModel(instance, disc, fixed);
        residual.solve();

        // === ACUMULAR Y GUARDAR ===
        int baseBoxes = bestBoxesPerLayer * bestLayers;
        int residualBoxes = residual.getPackedCount();
        int totalBoxes = baseBoxes + residualBoxes;

        double totalTimeMs = bestTime + residual.getSolveTimeMs();

        _lastBaseBoxes = baseBoxes;
        _lastResidualBoxes = residualBoxes;
        _lastTotalBoxes = totalBoxes;
        _lastTotalTimeMs = totalTimeMs;

        System.out.println("\n=== Totales Acumulados ===");
        System.out.println("Cajas por capas: " + baseBoxes);
        System.out.println("Cajas de relleno: " + residualBoxes);
        System.out.println("TOTAL cajas: " + totalBoxes);
        System.out.printf("Tiempo total: %.2f s\n", totalTimeMs / 1000.0);
    }

    // Getters para leer desde afuera si lo necesitás
    public static int getLastBaseBoxes() { return _lastBaseBoxes; }
    public static int getLastResidualBoxes() { return _lastResidualBoxes; }
    public static int getLastTotalBoxes() { return _lastTotalBoxes; }
    public static double getLastTotalTimeMs() { return _lastTotalTimeMs; }
}