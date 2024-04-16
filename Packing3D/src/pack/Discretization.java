package pack;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class Discretization
{
	private Instance _instance;
	
	private int[] _I;
	private int[] _J;
	private int[] _K;
	
	public Discretization(Instance instance)
	{
		_instance = instance;
		Set<Integer> set = new HashSet<Integer>();
		
		int max = Math.max(_instance.getL(), Math.max(_instance.getW(), _instance.getH()));
		
		for(int i=0; i <= max / _instance.getl(); ++i)
		for(int j=0; j <= max / _instance.getw(); ++j)
		for(int k=0; k <= max / _instance.geth(); ++k) if( i*_instance.getl() + j*_instance.getw() + k*_instance.geth() < max )
			set.add(i*_instance.getl() + j*_instance.getw() + k*_instance.geth());
		
		ArrayList<Integer> valores = new ArrayList<Integer>(set);
		Collections.sort(valores);
		
		int ci = (int)valores.stream().filter(v -> v < _instance.getL()).count();
		int cj = (int)valores.stream().filter(v -> v < _instance.getW()).count();
		int ck = (int)valores.stream().filter(v -> v < _instance.getH()).count();
		
		_I = new int[ci];
		_J = new int[cj];
		_K = new int[ck];
		
		for(int i=0; i<ci; ++i)
			_I[i] = valores.get(i);

		for(int j=0; j<cj; ++j)
			_J[j] = valores.get(j);

		for(int k=0; k<ck; ++k)
			_K[k] = valores.get(k);
		
//		System.out.print("I = [");
//		for(int i=0; i<ci; ++i)
//			System.out.print(_I[i] + " ");
//		System.out.println("]");
//		
//		System.out.print("J = [");
//		for(int i=0; i<cj; ++i)
//			System.out.print(_J[i] + " ");
//		System.out.println("]");
//		
//		System.out.print("K = [");
//		for(int i=0; i<ck; ++i)
//			System.out.print(_K[i] + " ");
//		System.out.println("]");
	}
	
	public int sizeI()
	{
		return _I.length;
	}
	
	public int sizeJ()
	{
		return _J.length;
	}
	
	public int sizeK()
	{
		return _K.length;
	}
	
	public int getx(int i)
	{
		return _I[i];
	}
	
	public int gety(int j)
	{
		return _J[j];
	}
	
	public int getz(int k)
	{
		return _K[k];
	}
}
