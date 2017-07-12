import java.util.Random;


/* ToDo: This class should implement a WeightFiller using an appropriate random distribution*/
public class RandomWeight implements WeightFiller {

	/*
	// Computes random numbers for weight initialization
	public float compute(int currWeight, int inSize, int outSize) 
	{
		Random rand = new Random();

		float r = (float) Math.sqrt(6/(inSize + outSize));
		float val = (rand.nextFloat() - (float)0.5)*2;
		return val * r;
	}
	*/

	public float compute(int currWeight, int inSize, int outSize) 
	{
		Random rand = new Random();

		float r = (float) Math.sqrt(inSize + outSize);
		float val = (rand.nextFloat() - (float)0.5)*2;
		return val * r;
	}
}
