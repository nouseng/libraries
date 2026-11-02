within Roll2RollDynamics.Examples;
model BeltDrivenCarriage
  "Bi-directional belt-driven carriage with pre-tension, variable-length stiffness and slack"
  extends Modelica.Icons.Example;

  // Components
  inner Roll2RollDynamics.WebWorld world(
    enableAnimation = true,
    animateWorld = false,
    animateGravity = false,
    nominalLength = 0.5,
    noSlip = true,
    allowSlack = true,
    g = 0,
    tension = 860,
    stressLow = 0,
    stressHigh = 2*world.stressNominal,
    bearingDamping = 0.005,
    webDampingTime = 0.006,
    web(EA = 1289984, density = 1200, thickness = 0.002, width = 0.032))
    "Megadyne AT10 belt (E = 20.156 GPa, A = 64 mm2, F_V = 860 N) and pulley damping B = 0.005 N.m.s/rad from Mehmood et al. (2026), Table 1"
    annotation(Placement(transformation(origin = {-190, 120}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed railMount(
    r = {1.468, 0, 0.051}, animation = false)
    "Linear guide midpoint at x = l_3/2 = 1.468 m, z = R = 0.051 m"
    annotation(Placement(transformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic rail(
    n = {1, 0, 0},
    useAxisFlange = true,
    animation = false,
    stateSelect = StateSelect.always,
    s(start = 0, fixed = true),
    v(start = 0, fixed = true))
    "Linear guide supporting the carriage along +x"
    annotation(Placement(transformation(origin = {-50, -40}, extent = {{-10, 10}, {10, -10}})));
  Modelica.Mechanics.Translational.Components.Damper guideDrag(d = 40)
    "Linear-guide viscous friction b = 40 N.s/m from Mehmood et al. (2026), Table 1"
    annotation(Placement(transformation(origin = {-48, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body carriage(
    m = 138,
    r_CM = {0, 0, 0},
    I_11 = 1.0,
    I_22 = 2.0,
    I_33 = 2.0,
    animation = false)
    "Cart and payload mass M = 138 kg from Mehmood et al. (2026), Tables 1 and 3"
    annotation(Placement(transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation leftClampOffset(
    r = {-0.188, 0, 0},
    animation = true,
    shapeType = "box",
    r_shape = {-0.188, 0, 0},
    lengthDirection = {1, 0, 0},
    widthDirection = {0, 1, 0},
    length = 0.376,
    width = 2*world.web.width,
    height = 0.045,
    color = {95, 100, 110})
    "Left belt clamp offset (-0.188 m, giving l_1 = 1.280 m at centre) and 0.376 m carriage body"
    annotation(Placement(transformation(origin = {-20, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rightClampOffset(
    r = {0.188, 0, 0},
    animation = false)
    "Right belt clamp offset (+0.188 m, giving l_2 = 1.280 m at centre)"
    annotation(Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.WebBoundary leftClamp(useSupport = true)
    "Left belt clamp on the carriage"
    annotation(Placement(transformation(origin = {-40, 40}, extent = {{10, -10}, {-10, 10}})));
  Roll2RollDynamics.Components.WebBoundary rightClamp(useSupport = true)
    "Right belt clamp on the carriage"
    annotation(Placement(transformation(origin = {40, 40}, extent = {{-10, -10}, {10, 10}})));

  Roll2RollDynamics.Components.Roller drivePulley(
    useFlange = true,
    noSlip = true,
    radius = 0.051 - 0.002/2,
    width = 0.032,
    density = 1368,
    xPosition = 0.0,
    zPosition = 0.0,
    fixedInitialAngle = false,
    fixedInitialWebState = true,
    entryAngleStart = Modelica.Constants.pi,
    exitAngleStart = 0)
    "Driven AT10 pulley (pitch radius R = 0.051 m, J_1 = 3.249e-4 kg.m2) at x = 0 m"
    annotation(Placement(transformation(origin = {-120, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idlerPulley(
    noSlip = true,
    radius = 0.051 - 0.002/2,
    width = 0.032,
    density = 1368,
    xPosition = 2.936,
    zPosition = 0.0,
    fixedInitialAngle = true,
    fixedInitialSpeed = true,
    fixedInitialWebState = true,
    entryAngleStart = 0,
    exitAngleStart = Modelica.Constants.pi)
    "Idler AT10 return pulley (pitch radius R = 0.051 m, J_2 = 3.249e-4 kg.m2) at x = l_3 = 2.936 m"
    annotation(Placement(transformation(origin = {120, 40}, extent = {{-10, -10}, {10, 10}})));

  Roll2RollDynamics.Components.Belt leftSpan(
    tensionNominal = 860,
    fixedInitialTension = true,
    fixedInitialMomentum = true)
    "Left upper span (K_1(0) = 1.008e6 N/m)"
    annotation(Placement(transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt rightSpan(
    tensionNominal = 860,
    fixedInitialTension = true,
    fixedInitialMomentum = true)
    "Right upper span (K_2(0) = 1.008e6 N/m)"
    annotation(Placement(transformation(origin = {80, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt returnSpan(
    tensionNominal = 860,
    fixedInitialTension = true,
    fixedInitialMomentum = true)
    "Lower return span (K_3 = 4.394e5 N/m)"
    annotation(Placement(transformation(origin = {0, -100}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed speedDrive(
    useSupport = false,
    exact = true)
    "Servo velocity source on the driven pulley"
    annotation(Placement(transformation(origin = {-150, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.CombiTimeTable speedProfile(
    smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
    table = [0.00, 0.0;
             0.04, 1.0;
             0.38, 28.0;
             0.58, 28.0;
             1.10, -36.0;
             1.60, -36.0;
             1.86, 36.0;
             1.961, 36.0;
             2.45, 0.0;
             2.50, 0.0])
    "Bi-directional shuttle up to 1.84 m/s (36 rad/s at R = 0.051 m) with fast return slack cutoff"
    annotation(Placement(transformation(origin = {-190, 80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(railMount.frame_b, rail.frame_a)
    annotation(Line(points = {{-80, -40}, {-60, -40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rail.frame_b, carriage.frame_a)
    annotation(Line(points = {{-40, -40}, {0, -40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rail.axis, guideDrag.flange_a)
    annotation(Line(points = {{-42, -46}, {-42, -55}, {-34, -55}, {-34, -70}, {-38, -70}}, color = {0, 127, 0}));
  connect(rail.support, guideDrag.flange_b)
    annotation(Line(points = {{-54, -46}, {-54, -55}, {-62, -55}, {-62, -70}, {-58, -70}}, color = {0, 127, 0}));
  connect(carriage.frame_a, leftClampOffset.frame_a)
    annotation(Line(points = {{0, -40}, {0, 0}, {-10, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(carriage.frame_a, rightClampOffset.frame_a)
    annotation(Line(points = {{0, -40}, {0, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(leftClampOffset.frame_b, leftClamp.frame_support)
    annotation(Line(points = {{-30, 0}, {-30, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rightClampOffset.frame_b, rightClamp.frame_support)
    annotation(Line(points = {{30, 0}, {30, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(speedProfile.y[1], speedDrive.w_ref)
    annotation(Line(points = {{-179, 80}, {-162, 80}}, color = {0, 0, 127}));
  connect(speedDrive.flange, drivePulley.flange_a)
    annotation(Line(points = {{-140, 80}, {-120, 80}, {-120, 40}}, color = {0, 0, 0}));
  connect(drivePulley.frame_b, leftSpan.frame_a)
    annotation(Line(points = {{-110, 40}, {-90, 40}}, color = {0, 128, 180}, thickness = 0.5));
  connect(leftSpan.frame_b, leftClamp.frame_a)
    annotation(Line(points = {{-70, 40}, {-50, 40}}, color = {0, 128, 180}, thickness = 0.5));
  connect(rightClamp.frame_a, rightSpan.frame_a)
    annotation(Line(points = {{50, 40}, {70, 40}}, color = {0, 128, 180}, thickness = 0.5));
  connect(rightSpan.frame_b, idlerPulley.frame_a)
    annotation(Line(points = {{90, 40}, {110, 40}}, color = {0, 128, 180}, thickness = 0.5));
  connect(idlerPulley.frame_b, returnSpan.frame_a)
    annotation(Line(points = {{130, 40}, {145, 40}, {145, -100}, {10, -100}}, color = {0, 128, 180}, thickness = 0.5));
  connect(returnSpan.frame_b, drivePulley.frame_a)
    annotation(Line(points = {{-10, -100}, {-145, -100}, {-145, 40}, {-130, 40}}, color = {0, 128, 180}, thickness = 0.5));

  annotation(
    experiment(StartTime = 0, StopTime = 2.5, Interval = 0.001, Tolerance = 1e-7),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-220, -140}, {160, 160}})),
    Documentation(figures = {
      Figure(title = "Carriage shuttle motion", identifier = "shuttleMotion", preferred = true,
        plots = {
          Plot(title = "Carriage position",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Position", unit = "m"),
            curves = {
              Curve(x = time, y = rail.s, legend = "Carriage position")}),
          Plot(title = "Carriage velocity and driven pulley surface velocity",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Velocity", unit = "m/s"),
            curves = {
              Curve(x = time, y = rail.v, legend = "Carriage velocity"),
              Curve(x = time, y = drivePulley.rollerVariables.surfaceVelocity, legend = "Driven pulley surface")})}),
      Figure(title = "Pre-tension, load sharing and slack cutoff", identifier = "overview", preferred = true,
        caption = "Simulated span tensions from Figure 3; the analytical compatibility references are plotted in the documentation image.",
        plots = {
          Plot(title = "Span tensions",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Span tension", unit = "N"),
            curves = {
              Curve(x = time, y = leftSpan.beltVariables.tension, legend = "Left span"),
              Curve(x = time, y = rightSpan.beltVariables.tension, legend = "Right span"),
              Curve(x = time, y = returnSpan.beltVariables.tension, legend = "Return span"),
              Curve(x = time, y = world.tension, legend = "Pre-tension")})}),
      Figure(title = "Upper span free lengths", identifier = "variableStiffness", preferred = true,
        caption = "Upper panel of Figure 4; the derived variable-length stiffness curves are in the documentation image.",
        plots = {
          Plot(title = "Upper span free lengths",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Free length", unit = "m"),
            curves = {
              Curve(x = time, y = leftSpan.beltVariables.freeLength, legend = "Left span L_1(x)"),
              Curve(x = time, y = rightSpan.beltVariables.freeLength, legend = "Right span L_2(x)")})})},
      info = "<html>
<h4>Bi-directional belt-driven carriage</h4>
<p>This example compares Roll to Roll Dynamics with the analytical linear
belt-drive model of Mehmood et al. [1] as a pre-tensioned carriage shuttles
back and forth along a rail.</p>
<p>Figure 1 shows the stage layout [1]. A driven pulley
(<code>drivePulley</code>) at <code>x = 0</code> m and an undriven return
pulley (<code>idlerPulley</code>) at <code>x = 2.936</code> m (pitch radius
<code>R = 51</code> mm, inertia <code>J_1 = J_2 = 3.249e-4</code>
kg&middot;m<sup>2</sup>, bearing damping <code>B = 0.005</code>
N&middot;m&middot;s/rad) carry a Megadyne AT10 belt pre-tensioned to
<code>world.tension = 860</code> N (<code>world.web.EA = 1289984</code> N,
<code>world.noSlip = true</code>, <code>world.allowSlack = true</code>). A
138 kg <code>carriage</code> slides on a prismatic <code>rail</code> with
40 N&middot;s/m viscous drag (<code>guideDrag</code>). Two
<a href=\"modelica://Roll2RollDynamics.Components.WebBoundary\">WebBoundary</a>
clamps (<code>leftClamp</code>, <code>rightClamp</code>) with
<code>useSupport = true</code> attach the belt ends to the carriage at
<code>&plusmn;0.188</code> m, splitting the loop into two variable-length upper
spans (<code>leftSpan</code>, <code>rightSpan</code>, each 1.280 m at centre)
and a 2.936 m lower <code>returnSpan</code>.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/BeltDrivenCarriage/carriage-layout.png\"
     width=\"600\"
     alt=\"2D schematic of the Mehmood et al. linear belt-drive stage with leftSpan, rightSpan, returnSpan and two WebBoundary clamps\">
<strong>Figure 1:</strong> Belt-driven carriage layout.</p>
<p>Figure 2 compares the carriage position and velocity (<code>rail.s</code>,
<code>rail.v</code>) with the inextensible-belt kinematic reference from
<code>drivePulley.rollerVariables.surfaceVelocity</code>. The carriage shuttles
forward to <code>+0.758</code> m, reverses to <code>-0.757</code> m and returns
to rest at <code>0.000</code> m with <code>-8.25</code> to
<code>+1.12</code> mm of elastic belt lag.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/BeltDrivenCarriage/shuttle-motion.png\"
     width=\"600\"
     alt=\"Carriage position and velocity compared with the inextensible-belt reference\">
<strong>Figure 2:</strong> Carriage position and velocity.</p>
<p>Because <code>idlerPulley</code> is undriven, <code>rightSpan</code> and
<code>returnSpan</code> act in series with combined branch length
<code>L_2(x) = L_right(x) + L_return + 1.5*pi*R</code> against the left branch
<code>L_1(x) = L_left(x) + 0.5*pi*R</code> (total belt length
<code>L_B = 5.816</code> m). With carriage load <code>F_c = M*a + b*v</code>
and idler rim drag <code>F_id = (J_2*alpha_idler + B*w_idler)/R</code>,
compatibility gives [1]:</p>
<blockquote><pre>
T_left   = max(0, T_0 - (L_2/L_B)*F_c - (L_ret/L_B)*F_id, -F_c)
T_right  = max(0, T_0 + (L_1/L_B)*F_c - (L_ret/L_B)*F_id, +F_c)
T_return = max(0, T_right + F_id)
</pre></blockquote>
<p>Figure 3 compares the simulated span tensions with these analytical
equations. During the gentler forward and reverse strokes (0&ndash;1.5 s), both
branches stay taut around the 860 N preload. During the fast return
acceleration (1.62&ndash;1.85 s), <code>leftSpan</code> unloads to its 0 N
slack cutoff while <code>rightSpan</code> and <code>returnSpan</code> carry the
full 3112 N pull. Raising <code>world.tension</code> from 860 to 1500 N keeps
all three spans taut throughout the return stroke.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/BeltDrivenCarriage/span-tensions.png\"
     width=\"600\"
     alt=\"Simulated span tensions compared with closed-form compatibility and slack predictions\">
<strong>Figure 3:</strong> Simulated and analytical span tensions.</p>
<p>Figure 4 shows the upper span free lengths and the resulting variable-length
stiffnesses <code>K_i(x) = EA/L_i(x)</code> from Eqs. (5)&ndash;(7) of [1]. As
the carriage approaches either pulley, the shorter upper span stiffens from
1007.8 kN/m at centre to 2469.0 kN/m, raising the active carriage stiffness from
1237.9 to 2049.4 kN/m; when <code>leftSpan</code> goes slack at 1.62 s, active
carriage stiffness drops to 247.4 kN/m (the series right-and-return branch
alone).</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/BeltDrivenCarriage/variable-stiffness.png\"
     width=\"600\"
     alt=\"Upper span free lengths and variable-length stiffnesses compared with Mehmood et al. (2026) Table 1\">
<strong>Figure 4:</strong> Upper span lengths and variable-length stiffness.</p>
<p>The table below compares Roll to Roll Dynamics with the analytical values of
Mehmood et al. [1]:</p>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Quantity</th><th>Mehmood et al. [1]</th><th>Roll to Roll Dynamics</th><th>Notes</th></tr>
<tr><td>Centered upper stiffness <code>K_1(0) = K_2(0)</code></td>
<td><code>1007.80</code> kN/m</td><td><code>1007.80</code> kN/m</td>
<td>Eq. (5), <code>EA/l_1</code> at <code>l_1 = 1.280</code> m</td></tr>
<tr><td>Return-span stiffness <code>K_3</code></td>
<td><code>439.37</code> kN/m</td><td><code>439.37</code> kN/m</td>
<td>Eq. (6), <code>EA/l_3</code> at <code>l_3 = 2.936</code> m</td></tr>
<tr><td>Peak upper stiffness at <code>|x| = 0.758</code> m</td>
<td><code>2469.0</code> kN/m</td><td><code>2469.0</code> kN/m</td>
<td>Eq. (5), <code>L_min = 0.522</code> m (Figure 4)</td></tr>
<tr><td>Centered carriage stiffness <code>K_eff(0)</code></td>
<td><code>1237.9</code> kN/m</td><td><code>1237.9</code> kN/m</td>
<td>Eq. (7) with pulley wraps (<code>1313.8</code> kN/m free spans only)</td></tr>
<tr><td>Taut stroke tension <code>T_left</code> (0&ndash;1.5 s)</td>
<td><code>337.0</code>&ndash;<code>1556.2</code> N</td><td><code>337.0</code>&ndash;<code>1557.8</code> N</td>
<td>Both branches taut around <code>F_V = 860</code> N (Figure 3)</td></tr>
<tr><td>Fast-return slack cutoff <code>T_left</code> (1.62&ndash;1.85 s)</td>
<td><code>0.0</code> N</td><td><code>0.0</code> N</td>
<td><code>K_eff</code> drops from <code>2049.4</code> to <code>247.4</code> kN/m</td></tr>
<tr><td>Fast-return peak tension <code>T_right</code> / <code>T_return</code></td>
<td><code>3104.5</code> / <code>3107.0</code> N</td><td><code>3107.1</code> / <code>3112.4</code> N</td>
<td>2.5 s RMS difference: <code>0.88</code>, <code>0.92</code>, <code>1.48</code> N (&lt;0.05%)</td></tr>
</table>
<h4>Limitations</h4>
<p>Tooth meshing occurs without slip (<code>world.noSlip = true</code>);
transverse belt vibration, tooth-ratcheting and gravity sag are omitted
(<code>world.g = 0</code>).</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://ris.utwente.nl/ws/portalfiles/portal/570143256/md-25-1925.pdf\">1</a>]
Y. Mehmood, W. Roozing, G. Svampa, F. Cannella and S. Cocuzza,
&quot;Performance-Driven Design Optimization of Linear Belt Drives Using
Analytical Modeling, Virtual Prototyping, and Experimental Validation&quot;,
ASME Journal of Mechanical Design, Vol. 148, No. 9, 2026, 093502, DOI
<a href=\"https://doi.org/10.1115/1.4072438\">10.1115/1.4072438</a>. Tables 1
and 3 stage geometry, Megadyne AT10 belt properties, carriage mass and damping;
Sec. 2.1&ndash;2.3, Eqs. (1) and (5)&ndash;(7), three-mass belt-drive equations
of motion, position-dependent span stiffnesses and series-parallel effective
carriage stiffness; Sec. 4, p. 10, 860 N pre-tension. Open access (CC BY 4.0),
accessed 2026-11-02.
<a href=\"https://ris.utwente.nl/ws/portalfiles/portal/570143256/md-25-1925.pdf\">Source</a>.</li>
</ul>
</html>"));
end BeltDrivenCarriage;
