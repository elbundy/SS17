/// ----------------------------------------------------------------------------------------
/// This interface describes functions needed for LossFunction
/// ----------------------------------------------------------------------------------------

public interface LossFunction 
{
	// computes loss(expected,real)
	float compute(Blob expected, Blob real);
	// computes loss'(expected,real)
	float[] derivative(Blob expected, Blob real);
}
