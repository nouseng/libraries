within MicroDuck.Utilities;
function policyNetwork "Evaluate the neural network of the walking policy"
  extends Modelica.Icons.Function;
  //Inputs
  input String weightFile "MAT file with the network weights";
  input Real[61] observation "Observation vector";
  //Outputs
  output Real[14] action "Policy action";
  external "C" duck_policy_network(weightFile, observation, action)
    annotation(
      Include = "#include \"duck_policy.h\"",
      IncludeDirectory = "modelica://MicroDuck/Resources/Include",
      LibraryDirectory = "modelica://Modelica/Resources/Library",
      Library = {"ModelicaIO", "ModelicaMatIO", "zlib"});

  annotation(Documentation(info = "<html>
<h4>Policy network</h4>
<p>Evaluates the network of the Microduck walking policy: <code>observation</code> is normalized with the mean and standard deviation in <code>weightFile</code>, then passes through three linear layers of 512, 256 and 128 units with ELU (exponential linear unit) activations and a linear output layer of 14 actions. The C implementation in <code>Resources/Include/duck_policy.h</code> reads the MATLAB v4 file exported from the ONNX (Open Neural Network Exchange) file of the policy [1] on the first call (or when <code>weightFile</code> changes) and caches the weights in memory.</p>
<h4>References</h4>
<p>[1] Pollen Robotics, microduck-policies, <code>velstand.onnx</code>, Apache-2.0, <a href=\"https://huggingface.co/pollen-robotics/microduck-policies\">huggingface.co/pollen-robotics/microduck-policies</a>. Accessed 2026-10-09.</p>
</html>"));
end policyNetwork;
