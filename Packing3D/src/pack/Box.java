package pack;

public class Box
{
	public enum Orientation { LW, WL, HW, HL, WH, LH };
	
	private static Instance _instance;
	private static Discretization _discretization;

	private Orientation _orientation;
	private int _i;
	private int _j;
	private int _k;
	
	public static void initialize(Instance instance, Discretization discretization)
	{
		_instance = instance;
		_discretization = discretization;
	}
	
	public Box(int i, int j, int k, Orientation orientation)
	{
		if( _instance == null )
			throw new RuntimeException("Box class not initialized!");

		_i = i;
		_j = j;
		_k = k;
		_orientation = orientation;
	}
	
	public int geti()
	{
		return _i;
	}
	
	public int getj()
	{
		return _j;
	}
	
	public int getk()
	{
		return _k;
	}
	
	public Orientation getOrientation()
	{
		return _orientation;
	}
	
	public int getLength()
	{
		if( _orientation == Orientation.LW || _orientation == Orientation.LH )
			return _instance.getl();

		if( _orientation == Orientation.WH || _orientation == Orientation.WL )
			return _instance.getw();
		
		return _instance.geth();
	}
	
	public int getWidth()
	{
		if( _orientation == Orientation.HL || _orientation == Orientation.WL )
			return _instance.getl();

		if( _orientation == Orientation.HW || _orientation == Orientation.LW )
			return _instance.getw();
		
		return _instance.geth();
	}
	
	public int getHeight()
	{
		if( _orientation == Orientation.HW || _orientation == Orientation.WH )
			return _instance.getl();

		if( _orientation == Orientation.HL || _orientation == Orientation.LH )
			return _instance.getw();
		
		return _instance.geth();
	}
	
	public int getx()
	{
		return _discretization.getx(_i);
	}
	
	public int gety()
	{
		return _discretization.gety(_j);
	}
	
	public int getz()
	{
		return _discretization.getz(_k);
	}
	
	public int getTop()
	{
		return getz() + getHeight();
	}
	
	public boolean fits()
	{
		return geti() >= 0 && getj() >= 0 && getk() >= 0 && getx() + getLength() <= _instance.getL() && gety() + getWidth() <= _instance.getW() && getz() + getHeight() <= _instance.getH();
	}
	
	public boolean contains(int i, int j, int k)
	{
		return _i <= i && _discretization.getx(i) < this.getx() + this.getLength()
			&& _j <= j && _discretization.gety(j) < this.gety() + this.getWidth()
			&& _k <= k && _discretization.getz(k) < this.getz() + this.getHeight();
	}
	
	public double floorSurface()
	{
		return this.getLength() * this.getWidth();
	}
	
	public boolean onTop(Box below)
	{
		return this.getz() == below.getz() + below.getHeight();
	}
	
	public double intersectionSurface(Box below)
	{
		if( this.onTop(below) == false )
			return 0;
		
		int maxx = Math.min(this.getx() + this.getLength(), below.getx() + below.getLength());
		int minx = Math.max(this.getx(), below.getx());
		
		if( minx >= maxx )
			return 0;
		
		int maxy = Math.min(this.gety() + this.getWidth(), below.gety() + below.getWidth());
		int miny = Math.max(this.gety(), below.gety());
		
		if( miny >= maxy )
			return 0;
		
		return (maxx - minx) * (maxy - miny);
	}
}
