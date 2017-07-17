import java.lang.Math;
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
		for(int i = 0; i<expected.getLength(); ++i){
			loss[i] = (expected.getValue(i) - real.getValue(i));
		}

		return loss;
	}

	// This function computes euclideanLoss(expected, real) and returns it
	public float compute(Blob expected, Blob real) 
	{
		float sum = 0;
		assert(expected.getLength()==real.getLength());
		for(int i = 0; i < expected.getLength(); i++){
			sum += Math.pow(expected.getValue(i) - real.getValue(i),2);
		}
		return (float)sum/(2*expected.getLength());
	}
}
