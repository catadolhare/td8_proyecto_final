package pack;

import java.util.*;
import ilog.concert.*;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.DoubleParam;
import pack.Box.Orientation;

public class BlockModel {

    private final Instance instance;
    private final Discretization disc;
    private final List<BlockType> types;
    private IloCplex cplex;
    private IloNumVar[][][][][] y;       // y[t][i][j][k][p]
    private BlockPlacement[][][][][] blk;
    private final Orientation[] ORS = Orientation.values();

    private boolean[][][] occupiedMask;
    private int totalBoxesPlaced = 0;
    private double objValue = 0.0;
    private String status = "Unknown";
    public String getStatus() { return status; }
    public boolean[][][] getOccupiedMask(){ return occupiedMask; }
    public int getTotalBoxesPlaced(){ return totalBoxesPlaced; }
    public double getObjValue(){ return objValue; }

    public BlockModel(Instance instance, Discretization disc, List<BlockType> types) {
        this.instance = instance;
        this.disc = disc;
        this.types = types;
    }

    // colocación concreta de un bloque
    private static class BlockPlacement {
        final BlockType t; final int i,j,k; final Orientation o;
        final int L,W,H;
        BlockPlacement(BlockType t,int i,int j,int k,Orientation o,Instance inst){
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
        boolean fits(Instance inst,Discretization d){
            int x0=d.getx(i),y0=d.gety(j),z0=d.getz(k);
            return x0+L<=inst.getL() && y0+W<=inst.getW() && z0+H<=inst.getH();
        }
        boolean overlaps(BlockPlacement b,Discretization d){
            int x1=d.getx(i),y1=d.gety(j),z1=d.getz(k);
            int x2=d.getx(b.i),y2=d.gety(b.j),z2=d.getz(b.k);
            return !(x1+L<=x2 || x2+b.L<=x1 || y1+W<=y2 || y2+b.W<=y1 || z1+H<=z2 || z2+b.H<=z1);
        }
        int baseArea(){ return L*W; }
        int intersectionArea(BlockPlacement below,Discretization d){
            if(d.getz(k)!=d.getz(below.k)+below.H) return 0;
            int x1=d.getx(i),x2=d.getx(below.i);
            int y1=d.gety(j),y2=d.gety(below.j);
            int xmax=Math.min(x1+L,x2+below.L);
            int xmin=Math.max(x1,x2);
            int ymax=Math.min(y1+W,y2+below.W);
            int ymin=Math.max(y1,y2);
            if(xmin>=xmax||ymin>=ymax) return 0;
            return (xmax-xmin)*(ymax-ymin);
        }
    }

    public void solve(int timeLimit) throws IloException {
        cplex = new IloCplex();
        cplex.setOut(null);
        cplex.setParam(DoubleParam.TimeLimit, timeLimit);

        int T=types.size(), I=disc.sizeI(), J=disc.sizeJ(), K=disc.sizeK(), P=ORS.length;
        y   = new IloNumVar[T][I][J][K][P];
        blk = new BlockPlacement[T][I][J][K][P];

        // Variables (solo para colocaciones que entran)
        for(int t=0;t<T;t++)
        for(int i=0;i<I;i++)
        for(int j=0;j<J;j++)
        for(int k=0;k<K;k++)
        for(int p=0;p<P;p++){
            BlockPlacement b=new BlockPlacement(types.get(t),i,j,k,ORS[p],instance);
            if(b.fits(instance,disc)){
                y[t][i][j][k][p]=cplex.boolVar("y_"+t+"_"+i+"_"+j+"_"+k+"_"+ORS[p]);
                blk[t][i][j][k][p]=b;
            }
        }

        // Objetivo: maximizar cajas (n por bloque)
        IloLinearNumExpr obj=cplex.linearNumExpr();
        for(int t=0;t<T;t++)
        for(int i=0;i<I;i++)
        for(int j=0;j<J;j++)
        for(int k=0;k<K;k++)
        for(int p=0;p<P;p++)
            if(y[t][i][j][k][p]!=null)
                obj.addTerm(types.get(t).n, y[t][i][j][k][p]);
        cplex.addMaximize(obj);

        // No solapamiento: cada celda cubierta a lo sumo 1 vez
        for(int ii=0;ii<I;ii++)
        for(int jj=0;jj<J;jj++)
        for(int kk=0;kk<K;kk++){
            IloLinearNumExpr lhs=cplex.linearNumExpr();
            for(int t=0;t<T;t++)
            for(int i=0;i<I;i++)
            for(int j=0;j<J;j++)
            for(int k=0;k<K;k++)
            for(int p=0;p<P;p++) if(y[t][i][j][k][p]!=null){
                BlockPlacement b=blk[t][i][j][k][p];
                int bx=disc.getx(i), by=disc.gety(j), bz=disc.getz(k);
                int ex=bx+b.L, ey=by+b.W, ez=bz+b.H;
                int px=disc.getx(ii), py=disc.gety(jj), pz=disc.getz(kk);
                if(px>=bx && px<ex && py>=by && py<ey && pz>=bz && pz<ez)
                    lhs.addTerm(1, y[t][i][j][k][p]);
            }
            cplex.addLe(lhs, 1);
        }

        // Estabilidad: 70% de base apoyada
        double thr=instance.getStabilityThreshold();
        for(int t=0;t<T;t++)
        for(int i=0;i<I;i++)
        for(int j=0;j<J;j++)
        for(int k=1;k<K;k++)
        for(int p=0;p<P;p++) if(y[t][i][j][k][p]!=null){
            BlockPlacement a=blk[t][i][j][k][p];
            IloLinearNumExpr lhs=cplex.linearNumExpr();
            lhs.addTerm(thr * a.baseArea(), y[t][i][j][k][p]);

            for(int tp=0;tp<T;tp++)
            for(int ip=0;ip<I;ip++)
            for(int jp=0;jp<J;jp++)
            for(int kp=0;kp<K;kp++)
            for(int pp=0;pp<P;pp++) if(y[tp][ip][jp][kp][pp]!=null){
                BlockPlacement b=blk[tp][ip][jp][kp][pp];
                int inter = a.intersectionArea(b, disc);
                if(inter>0) lhs.addTerm(-inter, y[tp][ip][jp][kp][pp]);
            }
            cplex.addLe(lhs, 0);
        }

        // Resolver y construir máscara
        cplex.solve();
        status = cplex.getStatus().toString();
        System.out.println("Status (bloques): " + status);

        if(cplex.getStatus()==IloCplex.Status.Optimal || cplex.getStatus()==IloCplex.Status.Feasible){
            objValue = cplex.getObjValue();
            occupiedMask = new boolean[disc.sizeI()][disc.sizeJ()][disc.sizeK()];
            totalBoxesPlaced = 0;

            for(int t=0;t<T;t++)
            for(int i=0;i<I;i++)
            for(int j=0;j<J;j++)
            for(int k=0;k<K;k++)
            for(int p=0;p<P;p++)
                if(y[t][i][j][k][p]!=null && cplex.getValue(y[t][i][j][k][p])>0.5){
                    totalBoxesPlaced += types.get(t).n;
                    markBlockOnMask(blk[t][i][j][k][p], occupiedMask, disc);
                }
        }
        cplex.end();
    }

    // marcado geométrico robusto
    private void markBlockOnMask(BlockPlacement b, boolean[][][] mask, Discretization d) {
        int I=d.sizeI(), J=d.sizeJ(), K=d.sizeK();
        int bx=d.getx(b.i), by=d.gety(b.j), bz=d.getz(b.k);
        int ex=bx+b.L,     ey=by+b.W,     ez=bz+b.H;
        for (int ii=0; ii<I; ii++)
        for (int jj=0; jj<J; jj++)
        for (int kk=0; kk<K; kk++) {
            int px=d.getx(ii), py=d.gety(jj), pz=d.getz(kk);
            if (px>=bx && px<ex && py>=by && py<ey && pz>=bz && pz<ez)
                mask[ii][jj][kk]=true;
        }
    }
}