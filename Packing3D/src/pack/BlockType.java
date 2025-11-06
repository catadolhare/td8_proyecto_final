package pack;

public class BlockType {

    // Cantidad de cajas agrupadas en cada dirección
    public final int a; // largo (multiplica l)
    public final int b; // ancho (multiplica w)
    public final int c; // alto  (multiplica h)

    // Total de cajas representadas por el bloque
    public final int n;

    // Dimensiones reales del bloque
    public final int Lb;
    public final int Wb;
    public final int Hb;

    // Constructor
    public BlockType(int a, int b, int c, Instance instance) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.n = a * b * c;

        // Calcula las dimensiones físicas del bloque
        this.Lb = a * instance.getl();
        this.Wb = b * instance.getw();
        this.Hb = c * instance.geth();
    }

    @Override
    public String toString() {
        return String.format("BlockType[%dx%dx%d -> (%d,%d,%d)]",
                a, b, c, Lb, Wb, Hb);
    }
}