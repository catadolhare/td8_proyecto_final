package console;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import pack.*;

public class EntryPoint {

    static final int L = 300, W = 280, H = 130; // contenedor fijo

    public static void main(String[] args) {
        int[][] cajas = {
            {30,30,85},{30,30,84},{40,35,75},{40,43,84},{40,40,75},{40,45,84},
            {40,40,76},{40,37,75},{41,37,75},{41,24,113},{45,35,61},{45,30,84},
            {45,32,84},{45,30,84},{50,45,109},{50,50,91},{55,40,80},{60,20,141},
            {62,39,92},{64,39,93},{69,40,95},{125,19,118},{130,55,75},{130,45,62},
            {135,29,47},{135,29,46},{138,30,47},{140,49,52},{160,33,46},{164,67,73},
            {170,85,45},{171,35,70},{185,55,55},{185,55,56},{190,175,45},{195,36,130}
        };

        String outputFile = "resultados.csv";
        try (FileWriter writer = new FileWriter(outputFile)) {
            writer.write("l,w,h,estrategia,cajas,tiempo_s,status\n");

            for (int[] c : cajas) {
                int l = c[0], w = c[1], h = c[2];
                System.out.println("\n===============================");
                System.out.println("Caja: " + l + "x" + w + "x" + h);
                System.out.println("===============================");

                Instance instance = new Instance(L, W, H, l, w, h);

                RunResult r = runInstance(instance, "bloques_dos_cajas", buildBlocksDosCajas(instance), 120);

                writer.write(l+","+w+","+h+","+r.estrategia+","+r.cajas+","+r.tiempo+","+r.status+"\n");
                writer.flush();

                System.out.println("→ Resultado = " + r.cajas + " cajas ("+r.estrategia+", "+r.tiempo+" s)");
            }

            System.out.println("\n✅ Resultados guardados en " + outputFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // --- helper para resultados ---
    private static class RunResult {
        final String estrategia, status;
        final int cajas;
        final double tiempo;
        RunResult(String estrategia, String status, int cajas, double tiempo) {
            this.estrategia = estrategia; this.status = status; this.cajas = cajas; this.tiempo = tiempo;
        }
    }

    private static RunResult runInstance(Instance instance, String estrategia, List<BlockType> tipos, int timePerBlockSec) {
        long t0 = System.currentTimeMillis();

        Discretization disc = new Discretization(instance);
        Box.initialize(instance, disc);

        BlockModelIterative bm = new BlockModelIterative(instance, disc, tipos);
        try {
            bm.solve(timePerBlockSec);
        } catch (Exception e) {
            return new RunResult(estrategia, "Exception:"+e.getMessage(), 0, (System.currentTimeMillis()-t0)/1000.0);
        }

        double secs = (System.currentTimeMillis() - t0) / 1000.0;
        return new RunResult(estrategia, bm.getStatus(), bm.getTotalBoxesPlaced(), secs);
    }

    // ===============================================================
    // BLOQUES DE DOS CAJAS (en cualquiera de los tres ejes) + unidad
    // ===============================================================
    private static List<BlockType> buildBlocksDosCajas(Instance instance) {
        List<BlockType> t = new ArrayList<>();

        // Bloques de 2 cajas
        t.add(new BlockType(3,1,1, instance));
        t.add(new BlockType(1,3,1, instance));
        t.add(new BlockType(1,1,3, instance));
        t.add(new BlockType(2,1,1, instance)); // 2 cajas - extendido a lo largo
        t.add(new BlockType(1,2,1, instance)); // 2 cajas - extendido a lo ancho
        t.add(new BlockType(1,1,2, instance)); // 2 cajas - apilado en altura

        // Unidad final
        t.add(new BlockType(1,1,1, instance)); // 1 caja

        return t;
    }
}