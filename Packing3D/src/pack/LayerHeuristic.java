package pack;

public class LayerHeuristic {

    public static void run(Instance instance) {
        Discretization disc = new Discretization(instance);

        int bestTotal = 0;
        Box.Orientation bestOrientation = null;
        int bestBoxesPerLayer = 0;
        int bestLayers = 0;
        double bestTime = 0.0;

        System.out.println("=== Heurística por Capas (versión simplificada) ===");
        System.out.println("Contenedor: L=" + instance.getL() + " W=" + instance.getW() + " H=" + instance.getH());
        System.out.println();

        for (Box.Orientation o : Box.Orientation.values()) {
            Box sample = new Box(0, 0, 0, o);
            int layerHeight = sample.getHeight();

            // Número de capas completas que caben en el contenedor (redondeado hacia abajo)
            int layers = instance.getH() / layerHeight;

            // Resolver solo UNA capa (por ejemplo, la capa inferior)
            CompleteModel2D model = new CompleteModel2D(instance, disc, o, 0);
            model.solve();

            int boxesInLayer = model.getSolutionCount();
            double solveTime = model.getSolveTime();

            // Multiplicar por la cantidad de capas idénticas que caben
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
                bestTime = solveTime;
            }
        }

        System.out.println("\n=== Resultado Final ===");
        System.out.println("Mejor orientación: " + bestOrientation);
        System.out.println("Cajas por capa: " + bestBoxesPerLayer);
        System.out.println("Cantidad de capas: " + bestLayers);
        System.out.println("Total de cajas en el contenedor: " + bestTotal);
        System.out.printf("Tiempo total de resolución (1 capa): %.2f segundos\n", bestTime / 1000);
    }
}
