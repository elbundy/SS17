/// ----------------------------------------------------------------------------------------
/// This class implements the euclidean loss, which is only used for OutputLayer
/// ----------------------------------------------------------------------------------------


/* ToDo: This class should implement the Euclidean-Loss used for Output-Layers */
public class EuclideanLoss implements LossFunction 
{
	// This function computes euclideanLoss'(expected, real) and returns it as an array
	// Defined as: (expected-real)^2
	public float[] derivative(Blob expected, Blob real) 
	{
		float[] loss=new float[expected.getLength()];

		return loss;
	}

	// This function computes euclideanLoss(expected, real) and returns it
	public float compute(Blob expected, Blob real) 
	{
		return (float)0.1337;
	}
}
