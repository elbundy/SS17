/// ----------------------------------------------------------------------------------------
/// This class implements the Output-Layer with weights and bias.
/// ----------------------------------------------------------------------------------------


/* ToDo: This class should implement the Output-Layer, which is the last layer of each Neural Network */
public class OutputLayer implements Layer
{
	// Loss function used for comparing output with predicted values
	LossFunction loss;
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
	
	Blob in;
   
	// TODO
	public Blob forward(Blob inputBlob)
	{
		in = new Blob(output.width);

		//Sum up inputs * weights
		for(int i=0; i<weights.width; i++){
			for(int j=0; j<weights.height; j++){
				in.addValue(j, inputBlob.getValue(i) * weights.getValue(i, j));
			}
		}
	
		//Add bias
		for(int i=0; i<bias.width; i++){
			in.addValue(i, bias.getValue(i));		
			output.setValue(i, func.compute(in.getValue(i)));
		}

		return output;
	}
	
	// TODO
	public Blob backward (Blob expectedOutput, Blob weightsBefore)
	{
		for(int i = 0; i<neuronDelta.width; ++i){
			float gstr = func.derivative(in.getValue(i));
			float bla = expectedOutput.getValue(i) - output.getValue(i);
			neuronDelta.setValue(i, gstr*bla);
		}
		return neuronDelta;
	}

	// TODO
	public void updateWeightsAndBias(Blob inputBlob, float learningRate)
	{
		for(int i=0; i<weights.width; i++){
			for(int j=0; j<weights.height; j++){
				weights.addValue(i, j, inputBlob.getValue(i)*learningRate*neuronDelta.getValue(j));
			}
		}
		
		for(int i = 0; i<bias.width; i++){
			bias.addValue(i, learningRate*neuronDelta.getValue(i));
		}
	}

	/** Constructor. */
	public OutputLayer(LossFunction loss, ActivationFunction func, WeightFiller fillerWeight, BiasFiller fillerBias, int in, int out)
	{
		output=new Blob(out);
		neuronDelta=new Blob(out);
		weights=new Blob(in,out);
		bias=new Blob(out);
		this.in = new Blob(out);

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
				weights.setValue(i, j,fillerWeight.compute(i*in+j, in, out));
			}
		}

		this.loss=loss;
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
