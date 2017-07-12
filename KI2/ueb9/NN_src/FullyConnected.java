
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

	Blob in;
	
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
	public Blob backward (Blob deltaBefore, Blob weightsBefore)
	{
		//System.out.println("Dims: " + deltaBefore.width + " " + deltaBefore.height + " " + weightsBefore.width + " " + weightsBefore.height);
		//System.out.println("Dims2: " + neuronDelta.width + " " + neuronDelta.height);
		for(int i = 0; i<neuronDelta.width; ++i){
			float gstr = func.derivative(in.getValue(i));
			//System.out.println("gstr: " + gstr);
			float sum = 0;
			for(int j = 0; j < weightsBefore.height; ++j){
				//System.out.println("deltaBefore: " + deltaBefore.getValue(j) + " weightsBefore: " + weightsBefore.getValue(i,j));
				sum += deltaBefore.getValue(j)*weightsBefore.getValue(i,j);
			}
			neuronDelta.setValue(i, gstr * sum);
			//System.out.println("NeuronDelta: " + neuronDelta.getValue(i));
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

	public FullyConnected(ActivationFunction func, WeightFiller fillerWeight,BiasFiller fillerBias , int in, int out)
	{
		output=new Blob(out);
		neuronDelta=new Blob(out);
		weights=new Blob(in,out);
		bias=new Blob(out);
		this.in = new Blob(output.width);
		
		
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
