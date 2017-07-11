
public class Testcases 
{
	static boolean testCaseTanh()
	{
		TanhActivation f=new TanhActivation();
		
		System.out.println("TanH Expected: 0.76159 Calculated: "+f.compute(1));
		System.out.println("TanH' Expected: 0.419974 Calculated: "+f.derivative(1));
		
		if(Math.abs(f.compute(1)-0.7615942)<0.001 && Math.abs(f.derivative(1)-0.41997433)<0.001)
		{
			return true;
		}
		
		return false;
	}
	
	static boolean testCaseEuclid()
	{
		EuclideanLoss f=new EuclideanLoss();
		
		Blob expected=new Blob(3);
		Blob real=new Blob(3);
		
		expected.setValue(0, 1.3f);
		expected.setValue(1, 1.6f);
		expected.setValue(2, 0.3f);
		
		real.setValue(0, 1.7f);
		real.setValue(1, 1.0f);
		real.setValue(2, 2.3f);
		
		float loss=f.compute(expected, real);
		float dervLoss[]=f.derivative(expected, real);
		float realLoss[]=new float[]{-0.4000001f, 0.6f, -2.0f};
		boolean wrong=false;
		
		System.out.println("EuclidLoss Expected: 1.5066 Calculated: "+loss+" Calculated*2: "+loss*2f);
		for(int i=0;i<dervLoss.length;i++)
		{
			if((realLoss[i]-dervLoss[i])>0.01 && (realLoss[i]-dervLoss[i]*0.5)>0.01  && (realLoss[i]*3f-dervLoss[i]*0.5)>0.01   && (realLoss[i]*3f-dervLoss[i])>0.01)
			{
				wrong=true;
			}
			System.out.println("EuclidLoss' Expected: "+realLoss[i]+" Calculated: "+dervLoss[i]+ " Calculated*0.5: "+dervLoss[i]*0.5);
		}
		
		
		if((Math.abs(loss-1.5066)<0.01 || Math.abs(loss*2f-1.5066)<0.01 || Math.abs(loss-1.5066*3f)<0.01 || Math.abs(loss*2f-1.5066*3f)<0.01 ) && wrong==false)
		{
			return true;
		}
		
		return false;
	}
	
	static boolean testCaseLinear()
	{
		LinearActivation f=new LinearActivation();
		
		System.out.println("Linear Expected: 2 Calculated: "+f.compute(2));
		System.out.println("Linear' Expected: 1 Calculated: "+f.derivative(2));
		
		if(Math.abs(f.compute(2)-2)<0.001 && Math.abs(f.derivative(2)-1)<0.001)
		{
			return true;
		}
		
		return false;
	}
	
	static boolean testCaseFullyConnectedForward()
	{
		Layer f=new FullyConnected(new TanhActivation(), new RandomWeight(), new ConstantBias(), 3, 5);
		Blob weights=new Blob(3,5);
		Blob bias=new Blob(5);
		
		for(int i=0;i<3*5;i++)
		{
			weights.setValue(i,(float) ((float)i*0.1));
		}
		for(int i=0;i<5;i++)
		{
			bias.setValue(i,(float) ((float)-i*0.05));
		}
		
		Blob inputBlob=new Blob(3);
		inputBlob.setValue(0, 0.6f);
		inputBlob.setValue(1, 0.2f);
		inputBlob.setValue(2, 0.4f);
		f.setWeights(weights);
		f.setBias(bias);
		
		Blob out=f.forward(inputBlob);
		float expectedOutput[]=new float[]{0.099668f, 0.3884727f, 0.6169093f, 0.7739084f, 0.8716722f};
		
		for(int i=0;i<out.getLength();i++)
		{
			System.out.println("Expected: "+expectedOutput[i]+" Calculated: "+out.getValue(i));
			if(Math.abs(out.getValue(i)-expectedOutput[i])>0.01)
			{
				return false;
			}
		}
		return true;
	}
	
	static boolean testCaseOutputBackward()
	{
		Layer f=new OutputLayer(new EuclideanLoss(), new TanhActivation(), new RandomWeight(), new ConstantBias(), 3, 2);
		Blob weights=new Blob(3,2);
		Blob bias=new Blob(2);
		Blob out = new Blob(2);
		
		for(int i=0;i<3*2;i++)
		{
			weights.setValue(i,(float) ((float)i*0.1));
		}
		for(int i=0;i<2;i++)
		{
			bias.setValue(i,(float) ((float)-i*0.05));
			out.setValue(i,(float) i);
		}

		Blob inputBlob=new Blob(3);
		inputBlob.setValue(0, 0.6f);
		inputBlob.setValue(1, 0.2f);
		inputBlob.setValue(2, 0.4f);
		f.setWeights(weights);
		f.setBias(bias);

		float expectedOutput[]=new float[]{-0.09868445f, 0.5277899f};
		
		f.forward(inputBlob);
		Blob delta=f.backward(out, null);
		
		for(int i=0;i<delta.getLength();i++)
		{
			System.out.println("Expected: "+expectedOutput[i]+" Calculated: "+delta.getValue(i)+" Calculated*0.5: "+delta.getValue(i)*0.5f);
			if(Math.abs(delta.getValue(i)-expectedOutput[i])>0.1 && Math.abs(delta.getValue(i)*0.5f-expectedOutput[i])>0.1)
			{
				return false;
			}
		}
		return true;
	}
	
	public static boolean testCaseAnd()
	{
		Datum[] dataAndLabel=DataReader.getAndData();
		
		Network n=new Network(new ConstantLearningRate(0.05f));
		
		n.add(new InputLayer(2));
		n.add(new OutputLayer(new EuclideanLoss(),new LinearActivation(), new RandomWeight(), new ConstantBias(), 2, 1));
		
		for(int j=0;j<10000;j++)
		{	
			for(int i=0;i<dataAndLabel.length;i++)
			{
				n.trainSimpleSGD(dataAndLabel[i].data, dataAndLabel[i].label);
			}
		}
		
		for(int i=0;i<dataAndLabel.length;i++)
		{
			Blob predictions[]=n.forward(dataAndLabel[i].data);
			for(int j=0;j<predictions[predictions.length-1].getLength();j++)
			{
				System.out.println("Expected: "+dataAndLabel[i].label.getValue(j)+" Calculated: "+predictions[predictions.length-1].getValue(j));
				if(Math.abs(predictions[predictions.length-1].getValue(j)-dataAndLabel[i].label.getValue(j))>0.49)
				{
					return false;
				}
			}
		}
	
		return true;
	}
	
	public static boolean testCaseXOr()
	{
		Datum[] dataAndLabel=DataReader.getXORData();
		
		Network n=new Network(new ConstantLearningRate(0.05f));
		
		n.add(new InputLayer(2));
		n.add(new FullyConnected(new TanhActivation(), new RandomWeight(), new ConstantBias(), 2, 3));
		n.add(new FullyConnected(new TanhActivation(), new RandomWeight(), new ConstantBias(), 3, 3));
		n.add(new OutputLayer(new EuclideanLoss(),new LinearActivation(), new RandomWeight(), new ConstantBias(), 3, 1));
		
		for(int j=0;j<20000;j++)
		{	
			for(int i=0;i<dataAndLabel.length;i++)
			{
				n.trainSimpleSGD(dataAndLabel[i].data, dataAndLabel[i].label);
			}
		}
		
		for(int i=0;i<dataAndLabel.length;i++)
		{
			Blob predictions[]=n.forward(dataAndLabel[i].data);
			for(int j=0;j<predictions[predictions.length-1].getLength();j++)
			{
				System.out.println("Expected: "+dataAndLabel[i].label.getValue(j)+" Calculated: "+predictions[predictions.length-1].getValue(j));
				if(Math.abs(predictions[predictions.length-1].getValue(j)-dataAndLabel[i].label.getValue(j))>0.05)
				{
					return false;
				}
			}
		}
	
		return true;
	}
	
	public static boolean testCase2In4Out()
	{
		Datum[] dataAndLabel=DataReader.get2In4OutDataset();
		
		Network n=new Network(new ConstantLearningRate(0.005f));
		
		n.add(new InputLayer(2));
		n.add(new FullyConnected(new TanhActivation(), new RandomWeight(), new ConstantBias(), 2, 6));
		n.add(new FullyConnected(new TanhActivation(), new RandomWeight(), new ConstantBias(), 6, 5));
		n.add(new OutputLayer(new EuclideanLoss(),new LinearActivation(), new RandomWeight(), new ConstantBias(), 5, 4));
		
		for(int j=0;j<20000;j++)
		{	
			for(int i=0;i<dataAndLabel.length;i++)
			{
				n.trainSimpleSGD(dataAndLabel[i].data, dataAndLabel[i].label);
			}
		}
		
		for(int i=0;i<dataAndLabel.length;i++)
		{
			Blob predictions[]=n.forward(dataAndLabel[i].data);
			System.out.println("Datum["+i+"]:");
			for(int j=0;j<predictions[predictions.length-1].getLength();j++)
			{
				System.out.println("Expected["+j+"]: "+dataAndLabel[i].label.getValue(j)+" Calculated["+j+"]: "+predictions[predictions.length-1].getValue(j));
				if(Math.abs(predictions[predictions.length-1].getValue(j)-dataAndLabel[i].label.getValue(j))>0.05)
				{
					return false;
				}
			}
		}
	
		return true;
	}
	
	public static int doTestcases()
	{
		System.out.println("-------------------------------------------------------------");
		System.out.println("Testing TanH-Activation-Method...");
		boolean tanh=testCaseTanh();
		System.out.println("testCaseTanh success: "+tanh+ " (Maximum allowed deviation of 0.01)");
		System.out.println("-------------------------------------------------------------");
		System.out.println("Testing Linear-Activation-Method...");
		boolean linear=testCaseLinear();
		System.out.println("testCaseLinear success: "+linear+ " (Maximum allowed deviation of 0.001)");
		System.out.println("-------------------------------------------------------------");
		System.out.println("Testing EuclideanLoss-Method...");
		boolean euclid=testCaseEuclid();
		System.out.println("testCaseEuclid success: "+euclid+ " (Maximum allowed deviation of 0.001)");
		
		System.out.println("-------------------------------------------------------------");
		System.out.println("Testing FullyConnected-Forward-Method...");
		boolean fully=testCaseFullyConnectedForward();
		System.out.println("testCaseFullyConnectedForward success: "+fully+ " (Maximum allowed deviation of 0.01)");
		System.out.println("-------------------------------------------------------------");
		System.out.println("Testing OutputLayer-Backward-Method using EuclideanLoss...");
		boolean outputLayer=testCaseOutputBackward();
		System.out.println("testCaseOutputBackward success: "+outputLayer+ " (Maximum allowed deviation of 0.1)");
		
		System.out.println("-------------------------------------------------------------");
		System.out.println("Training Network on simple AND problem using only Input- and Output-layer...");
		boolean andSet=testCaseAnd();
		System.out.println("testCaseAnd success: "+andSet+ " (Maximum allowed deviation of 0.5)");
		System.out.println("-------------------------------------------------------------");
		System.out.println("Training Network on simple XOR problem...");
		boolean xor=testCaseXOr();
		System.out.println("testCaseXOr success: "+xor+ " (Maximum allowed deviation of 0.05)");
		System.out.println("-------------------------------------------------------------");
		System.out.println("Training Network on advanced problem with 4 outputs...");
		boolean in2Out4=testCase2In4Out();
		System.out.println("testCase2In4Out success: "+in2Out4+ " (Maximum allowed deviation of 0.05 per Output-Neuron)");
		System.out.println("-------------------------------------------------------------");

		int overallSuccessfull=(fully) ? 1 : 0;
		overallSuccessfull+=(andSet) ? 1 : 0;
		overallSuccessfull+=(xor) ? 1 : 0;
		overallSuccessfull+=(in2Out4) ? 1 : 0;
		overallSuccessfull+=(outputLayer) ? 1 : 0;
		overallSuccessfull+=(linear) ? 1 : 0;
		overallSuccessfull+=(tanh) ? 1 : 0;
		overallSuccessfull+=(euclid) ? 1 : 0;
		
		System.out.println("Overall correct testcases: "+overallSuccessfull+" of 8");
		System.out.println("----------------------------------");
		
		return overallSuccessfull;
	}
}
