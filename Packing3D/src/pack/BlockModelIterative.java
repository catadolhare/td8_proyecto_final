package pack;

import java.util.*;
import ilog.concert.*;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.DoubleParam;
import pack.Box.Orientation;

public class BlockModelIterative {

    private final Instance instance;
    private final Discretization disc;
    private final List<BlockType> types;          // lista ordenada grande → chico
    private final Orientation[] ORS = Orientation.values();

    private boolean[][][] occupiedMask;           // máscara global (acumula todas las iteraciones)
    private final List<PlacedBlock> placed = new ArrayList<>();

    private IloCplex cplex;
    private String status = "Unknown";
    private double objValue = 0.0;
    private int totalBoxesPlaced = 0;

    public String getStatus() { return status; }
    public double getObjValue() { return objValue; }
    public int getTotalBoxesPlaced() { return totalBoxesPlaced; }
    public boolean[][][] getOccupiedMask() { return occupiedMask; }
    public List<PlacedBlock> getPlacedBlocks(){ return Collections.unmodifiableList(placed); }

    public BlockModelIterative(Instance instance, Discretization disc, List<BlockType> types) {
        this.instance = instance;
        this.disc = disc;
        // copiamos y aseguramos orden por n descendente
        List<BlockType> sorted = new ArrayList<>(types);
        sorted.sort((a,b)->Integer.compare(b.n, a.n));
        this.types = sorted;
        this.occupiedMask = new boolean[disc.sizeI()][disc.sizeJ()][disc.sizeK()];
    }

    // ------------ Representación de colocación ------------
    public static class PlacedBlock {
        public final BlockType type;
        public final int i, j, k;
        public final Orientation o;
        public final int L, W, H;   // dimensiones físicas del bloque
        public PlacedBlock(BlockType t, int i, int j, int k, Orientation o, int L, int W, int H) {
            this.type = t; this.i = i; this.j = j; this.k = k; this.o = o; this.L = L; this.W = W; this.H = H;
        }
        @Override public String toString() {
            return "PlacedBlock{" + type + " @ ("+i+","+j+","+k+") "+o+"}";
        }
    }

    private static class BlockPlacement {
        final BlockType t; final int i,j,k; final Orientation o;
        final int L,W,H;
        BlockPlacement(BlockType t,int i,int j,int k,Orientation o){
            this.t=t; this.i=i; this.j=j; this.k=k; this.o=o;
            switch(o){
                case LW: default: L=t.Lb; W=t.Wb; H=t.Hb; break;
                case WL: L=t.Wb; W=t.Lb; H=t.Hb; break;
                case HW: L=t.Hb; W=t.Wb; H=t.Lb; break;
                case HL: L=t.Hb; W=t.Lb; H=t.Wb; break;
                case WH: L=t.Wb; W=t.Hb; H=t.Lb; break;
                case LH: L=t.Lb; W=t.Hb; H=t.Wb; break;
            }
        }
        boolean fits(Instance inst, Discretization d){
            int x0=d.getx(i), y0=d.gety(j), z0=d.getz(k);
            return x0+L<=inst.getL() && y0+W<=inst.getW() && z0+H<=inst.getH();
        }
        int baseArea(){ return L*W; }
        // intersección de base de "a" con techo de "b" (si b está justo debajo de a)
        static int intersectionArea(BlockPlacement a, BlockPlacement b, Discretization d){
            if (d.getz(a.k) != d.getz(b.k) + b.H) return 0;
            int ax1=d.getx(a.i), ax2=ax1+a.L, ay1=d.gety(a.j), ay2=ay1+a.W;
            int bx1=d.getx(b.i), bx2=bx1+b.L, by1=d.gety(b.j), by2=by1+b.W;
            int dx=Math.max(0, Math.min(ax2,bx2)-Math.max(ax1,bx1));
            int dy=Math.max(0, Math.min(ay2,by2)-Math.max(ay1,by1));
            return dx*dy;
        }
    }

    // ------------ Heurística iterativa: grande → chico ------------
    public void solve(int timePerBlockSeconds) throws IloException {
        objValue = 0.0;
        totalBoxesPlaced = 0;
        status = "Started";

        for (BlockType t : types) {
            // 1) Modelo solo para el bloque t
            cplex = new IloCplex();
            cplex.setOut(null);
            cplex.setParam(DoubleParam.TimeLimit, timePerBlockSeconds);

            int I=disc.sizeI(), J=disc.sizeJ(), K=disc.sizeK(), P=ORS.length;
            IloNumVar[][][][] x = new IloNumVar[I][J][K][P];
            BlockPlacement[][][][] bp = new BlockPlacement[I][J][K][P];
            double[][][][] fixedSupport = new double[I][J][K][P]; // soporte constante de bloques ya colocados

            // 1.a) Variables sólo donde entra el bloque t, no solapa máscara previa
            for (int i=0;i<I;i++)
            for (int j=0;j<J;j++)
            for (int k=0;k<K;k++)
            for (int p=0;p<P;p++){
                BlockPlacement b = new BlockPlacement(t,i,j,k,ORS[p]);
                if (!b.fits(instance,disc)) continue;
                if (overlapsOccupiedMask(b)) continue;
                x[i][j][k][p]  = cplex.boolVar("x_"+i+"_"+j+"_"+k+"_"+ORS[p]);
                bp[i][j][k][p] = b;
                fixedSupport[i][j][k][p] = supportFromAlreadyPlaced(b);
            }

            // 1.b) Objetivo: maximizar #cajas del bloque t
            IloLinearNumExpr obj = cplex.linearNumExpr();
            for (int i=0;i<I;i++)
            for (int j=0;j<J;j++)
            for (int k=0;k<K;k++)
            for (int p=0;p<P;p++)
                if (x[i][j][k][p]!=null)
                    obj.addTerm(t.n, x[i][j][k][p]);
            cplex.addMaximize(obj);

            // 1.c) No solapamiento dentro de esta iteración + evita pisar máscara previa (ya filtrado)
            for (int ii=0;ii<I;ii++)
            for (int jj=0;jj<J;jj++)
            for (int kk=0;kk<K;kk++){
                IloLinearNumExpr lhs = cplex.linearNumExpr();
                for (int i=0;i<I;i++)
                for (int j=0;j<J;j++)
                for (int k=0;k<K;k++)
                for (int p=0;p<P;p++) if (x[i][j][k][p]!=null){
                    if (cellCoveredBy(ii,jj,kk, bp[i][j][k][p]))
                        lhs.addTerm(1, x[i][j][k][p]);
                }
                cplex.addLe(lhs, 1);
            }

            // 1.d) Estabilidad (70%): soporte de (i) bloques previos (constante) + (ii) bloques de esta iteración
            double thr = instance.getStabilityThreshold();
            for (int i=0;i<I;i++)
            for (int j=0;j<J;j++)
            for (int k=1;k<K;k++) // k=0: piso, no necesita soporte
            for (int p=0;p<P;p++) if (x[i][j][k][p]!=null){
                BlockPlacement a = bp[i][j][k][p];
                IloLinearNumExpr lhs = cplex.linearNumExpr();
                lhs.addTerm(thr * a.baseArea(), x[i][j][k][p]);

                // soporte de bloques de esta misma iteración:
                for (int ip=0;ip<I;ip++)
                for (int jp=0;jp<J;jp++)
                for (int kp=0;kp<K;kp++)
                for (int pp=0;pp<P;pp++) if (x[ip][jp][kp][pp]!=null){
                    BlockPlacement b = bp[ip][jp][kp][pp];
                    int inter = BlockPlacement.intersectionArea(a, b, disc);
                    if (inter>0)
                        lhs.addTerm(-inter, x[ip][jp][kp][pp]);
                }

                // agregamos la constante del soporte previo correctamente
                IloNumExpr stabExpr = lhs;
                if (fixedSupport[i][j][k][p] > 0) {
                    stabExpr = cplex.sum(stabExpr, cplex.constant(-fixedSupport[i][j][k][p]));
                }
                cplex.addLe(stabExpr, 0);
            }

            // 1.e) Resolver este bloque
            cplex.solve();
            String st = cplex.getStatus().toString();
            System.out.println("Status (bloque "+t+") : " + st);

            if (cplex.getStatus()==IloCplex.Status.Optimal || cplex.getStatus()==IloCplex.Status.Feasible) {
                // 1.f) Recuperar solución, actualizar máscara y contadores
                double localObj = cplex.getObjValue();
                objValue += localObj; // suma de (t.n * cantidad colocada)
                int placedCount = 0;

                for (int i=0;i<I;i++)
                for (int j=0;j<J;j++)
                for (int k=0;k<K;k++)
                for (int p=0;p<P;p++)
                    if (x[i][j][k][p]!=null && cplex.getValue(x[i][j][k][p])>0.5) {
                        BlockPlacement b = bp[i][j][k][p];
                        placed.add(new PlacedBlock(t, i, j, k, ORS[p], b.L, b.W, b.H));
                        totalBoxesPlaced += t.n;
                        placedCount++;
                        markBlockOnMask(b);
                    }

                System.out.println("  → Colocados " + placedCount + " bloques de " + t + " (" + (placedCount * t.n) + " cajas)");
            }

            cplex.end();
        }

        status = "OptimalOrBestFoundPerIteration";
    }

    // ------------ Soporte y solapamiento respecto a lo ya colocado ------------
    private boolean overlapsOccupiedMask(BlockPlacement b) {
        int I=disc.sizeI(), J=disc.sizeJ(), K=disc.sizeK();
        int bx=disc.getx(b.i), by=disc.gety(b.j), bz=disc.getz(b.k);
        int ex=bx+b.L, ey=by+b.W, ez=bz+b.H;
        for (int ii=0; ii<I; ii++)
        for (int jj=0; jj<J; jj++)
        for (int kk=0; kk<K; kk++) {
            if (!occupiedMask[ii][jj][kk]) continue;
            int px=disc.getx(ii), py=disc.gety(jj), pz=disc.getz(kk);
            if (px>=bx && px<ex && py>=by && py<ey && pz>=bz && pz<ez)
                return true;
        }
        return false;
    }

    private double supportFromAlreadyPlaced(BlockPlacement a) {
        double sup = 0.0;
        for (PlacedBlock pb : placed) {
            BlockPlacement b = new BlockPlacement(pb.type, pb.i, pb.j, pb.k, pb.o);
            int inter = BlockPlacement.intersectionArea(a, b, disc);
            sup += inter;
        }
        return sup;
    }

    private boolean cellCoveredBy(int ii,int jj,int kk, BlockPlacement b) {
        int bx=disc.getx(b.i), by=disc.gety(b.j), bz=disc.getz(b.k);
        int ex=bx+b.L, ey=by+b.W, ez=bz+b.H;
        int px=disc.getx(ii), py=disc.gety(jj), pz=disc.getz(kk);
        return (px>=bx && px<ex && py>=by && py<ey && pz>=bz && pz<ez);
    }

    private void markBlockOnMask(BlockPlacement b) {
        int I=disc.sizeI(), J=disc.sizeJ(), K=disc.sizeK();
        int bx=disc.getx(b.i), by=disc.gety(b.j), bz=disc.getz(b.k);
        int ex=bx+b.L, ey=by+b.W, ez=bz+b.H;
        for (int ii=0; ii<I; ii++)
        for (int jj=0; jj<J; jj++)
        for (int kk=0; kk<K; kk++) {
            int px=disc.getx(ii), py=disc.gety(jj), pz=disc.getz(kk);
            if (px>=bx && px<ex && py>=by && py<ey && pz>=bz && pz<ez)
                occupiedMask[ii][jj][kk] = true;
        }
    }
}