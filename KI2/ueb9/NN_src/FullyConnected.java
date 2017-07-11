
/** class FullyConnected.
*/
public class FullyConnected implements Layer
{
	// Weights from all neurons of layer before to all neurons of current layer
	Blob weights;
	// Activation function for neurons
	ActivationFunction func;
	// The output of each neuron after applying activation functions etc.
	Blob output;
	// neuronDelta/gradient of each neuron
	Blob neuronDelta;
	// Bias for each neuron
	Blob bias;
	
	// TODO
	public Blob forward(Blob inputBlob)
	{

		return output;
	}

	// TODO
	public Blob backward (Blob deltaBefore, Blob weightsBefore)
	{

		return neuronDelta;
	}

	// TODO
	public void updateWeightsAndBias(Blob inputBlob, float learningRate)
	{

	}

	public FullyConnected(ActivationFunction func, WeightFiller fillerWeight,BiasFiller fillerBias , int in, int out)
	{
		output=new Blob(out);
		neuronDelta=new Blob(out);
		weights=new Blob(in,out);
		bias=new Blob(out);
		
		
		for(int i=0;i<bias.getLength();i++)
		{
			// Set Bias-Value of Neuron i
			bias.setValue(i,fillerBias.compute(i, in, out));
		}
		
		for(int i=0;i<in;i++)
		{
			for(int j=0;j<out;j++)
			{
				// Set initial weight between Neuron i and Neuron j
				weights.setValue(i,j,fillerWeight.compute(i*in+j, in, out));
			}
		}
		this.func=func;
		
	}

	public Blob getWeights() {
		return weights;
	}
	
	public void setWeights(Blob weights) {
		this.weights=weights;
	}

	public void setBias(Blob bias) {
		this.bias=bias;
		
	}


}