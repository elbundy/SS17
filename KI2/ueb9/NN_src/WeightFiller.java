/// ----------------------------------------------------------------------------------------
/// This interface describes the methods of initialization of weights.
/// ----------------------------------------------------------------------------------------

public interface WeightFiller
{
	public float compute(int currWeight, int inSize, int outSize);
}
