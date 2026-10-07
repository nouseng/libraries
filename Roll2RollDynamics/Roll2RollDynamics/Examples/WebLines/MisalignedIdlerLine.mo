within Roll2RollDynamics.Examples.WebLines;
model MisalignedIdlerLine
  "Converting line with tension control and fixed idler misalignment"
     extends Modelica.Icons.Example;
  // Parameters
  parameter Modelica.Units.SI.Time startupDuration(min=Modelica.Constants.small) = 2
    "Time for the driven surfaces to reach the line speed"
    annotation(Dialog(group = "Operating conditions"));
  final parameter Real normalEntry(unit = "1") = 0
    "Zero roll-relative entry skew for normal entry";
protected
  // Parameters
  // Layout derived from the line, not settable
  final parameter Modelica.Units.SI.Angle WebWorldAngle = 0.6
    "Inclination of the web spans at the rolls";
  final parameter Modelica.Units.SI.Radius unwindRadius = 0.40
    "Initial unwind roll radius";
  final parameter Modelica.Units.SI.Radius rewindRadius = 0.10
    "Initial rewind roll radius";
  final parameter Modelica.Units.SI.Length zBelt = world.rollRadius + world.web.thickness/2 "Belt centerline radius";
  final parameter Modelica.Units.SI.Length xW = zBelt*sin(WebWorldAngle)
    "Machine-direction offset of a tangency point from the roll centre";
  final parameter Modelica.Units.SI.Length zW = zBelt*cos(WebWorldAngle)
    "Height of a tangency point above the roll centre";
  final parameter Modelica.Units.SI.Length xMid = world.spanLength/2 "Dancer x-coordinate";
  final parameter Modelica.Units.SI.Length xMid2 = 3*world.spanLength/2 "Guide x-coordinate";
  final parameter Modelica.Units.SI.Length xStart = -0.5 "Unwind roll center x-position";
  final parameter Modelica.Units.SI.Length xEnd = 2*world.spanLength + 0.5 "Rewind roll center x-position";
  final parameter Modelica.Units.SI.Length zDeflected = zW + (world.rollRadius - sin(WebWorldAngle)*(xMid - xW))/cos(WebWorldAngle) "Deflected roll centre height, shared by the dancer and the guide";
  final parameter Modelica.Units.SI.Length zUt0 = zW + tan(WebWorldAngle)*(xStart + xW) "Unwind top height";
  final parameter Modelica.Units.SI.Length zUnwindCenter = zUt0 - unwindRadius "Unwind center height";
  final parameter Modelica.Units.SI.Length zRt0 = zW - tan(WebWorldAngle)*(xEnd - 2*world.spanLength - xW) "Rewind top height";
  final parameter Modelica.Units.SI.Length zRewindCenter = zRt0 - rewindRadius "Rewind center height";

public

  // Variables with Binding Equations
  // State of the span entering the misaligned roller, for later span analysis
  output Modelica.Units.SI.Length idlerSpanLength = belt3.beltVariables.freeLength
    "Free length of the span running into the misaligned roller";
  output Modelica.Units.SI.Force idlerSpanTension = belt3.tension
    "Tension that span carries";
  output Modelica.Units.SI.Stress idlerSpanStress =
    belt3.tension/(world.web.thickness*world.web.width)
    "Mean machine-direction stress in that span";
  output Real idlerSpanAspect(unit = "1") = belt3.beltVariables.freeLength/world.web.width
    "Span length over web width, which sets the buckling mode the span can take";
  output Real idlerSpanSkewSin(unit = "1") = belt3.beltVariables.spanSkewSin
    "Sine of the angle that span makes across the machine";
  output Real idlerSpanAxisSkew(unit = "1") = idler.entrySkewSin
    "Entering span direction along the idler axis, zero at normal entry";
  output Modelica.Units.SI.Position idlerLateralOffset = idler.rollerVariables.lateralPosition
    "Cross-machine offset the web has walked to at the misaligned roller";

  // Components
  // The line states its values on the WebWorld below; only the tension loop and
  // the dancer suspension, which are Modelica Standard Library parts rather
  // than line defaults, are set at their own components.
  inner Roll2RollDynamics.WebWorld world(
    web(thickness = 125e-6, width = 0.6, density = 1390, EA = 270e3),
    tension = 400,
    lineSpeed = 2.0,
    rollRadius = 0.075,
    coreRadius = 0.076,
    lateralDynamics = true, rollColor = {0, 120, 0})
    "Multibody world and shared line defaults, set to a 125 um polyester film line"
    annotation(Placement(transformation(extent = {{-250, 130}, {-230, 150}})));

  // Wound rolls; the connected speed sources initialize their shaft angles.
  Roll2RollDynamics.Components.Winder unwindDrum(
    winding = -1,
    radius0 = unwindRadius,
    xPosition = xStart,
    zPosition = zUnwindCenter,
    rotationDirection = 1, fixedInitialWebState=true)
    annotation(Placement(transformation(origin = {-240, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Winder rewindDrum(
    winding = +1,
    radius0 = rewindRadius,
    xPosition = xEnd,
    zPosition = zRewindCenter,
    rotationDirection = 1, fixedInitialWebState=true)
    annotation(Placement(transformation(origin = {240, 65}, extent = {{10, -10}, {-10, 10}})));

  // Rollers / idlers / dancer / guide
  Roll2RollDynamics.Components.Roller infeed(
    useFlange = true,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = 0,
    zPosition = 0)
    annotation(Placement(transformation(origin = {-160, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller dancer(
    useSupport = true,
    rotationDirection = -1,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    rollColor = {0, 128, 255})
    annotation(Placement(transformation(origin = {-80, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler(
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = world.spanLength,
    zPosition = 0,
    yaw = 0.004,
    useDynamicColor = true,
    FnThreshold = 440,
    FnSharpness = 30)
    annotation(Placement(transformation(origin = {0, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller guide(fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = xMid2,
    zPosition = zDeflected, rotationDirection = -1)
    annotation(Placement(transformation(origin = {80, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller outfeed(
    useFlange = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = false,
    xPosition = 2*world.spanLength,
    zPosition = 0)
    annotation(Placement(transformation(origin = {160, 65}, extent = {{-10, -10}, {10, 10}})));
  // The speed source initializes the shaft angle and prescribes its speed.
  // External dancer support
  Modelica.Mechanics.MultiBody.Parts.Fixed dancerBase(
    r = {xMid, 0, zDeflected},
    animation = false) "Nominal dancer support position"
    annotation(Placement(transformation(origin = {-80, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic dancerPrismatic(
    n = {0, 0, 1},
    useAxisFlange = true,
    stateSelect = StateSelect.always,
    s(start = 0, fixed = true),
    v(start=0,fixed=true)) "External vertical dancer degree of freedom"
    annotation(Placement(transformation(origin = {-80, 8.93}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
  Modelica.Mechanics.Translational.Components.SpringDamper dancerSuspension(
    c = 15000,
    d = 400,
    s_rel0 = (2*world.tension*sin(WebWorldAngle)
      - world.g*(dancer.density*Modelica.Constants.pi*
        (dancer.radius^2-(dancer.radius-dancer.wallThickness)^2)*dancer.width
        + world.web.density*world.web.thickness*world.web.width*
          (world.spanLength/2 + dancer.radius*2*WebWorldAngle)))/dancerSuspension.c) "External dancer restoring load and damping"
    annotation(Placement(transformation(origin = {-50, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -270)));

  // Web spans
  Roll2RollDynamics.Components.Belt belt1 annotation(
Placement(transformation(origin = {-200, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt belt2 annotation(Placement(transformation(origin = {-120, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt belt3 annotation(Placement(transformation(origin = {-40, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt belt4 annotation(Placement(transformation(origin = {40, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt belt5 annotation(Placement(transformation(origin = {120, 65}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt belt6 annotation(Placement(transformation(origin = {200, 65}, extent = {{-10, -10}, {10, 10}})));

  // PI tension controller for this speed-constrained line. Increasing infeed
  // torque raises the slow downstream tension response, requiring positive gain.
  Modelica.Blocks.Sources.Constant tensionReference(k = world.tension, y(unit = "N"))
    "Baseline tension setpoint"
    annotation(Placement(transformation(origin = {-80, 150}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Continuous.LimPID controller(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = 0.0016,
    Ti = 0.5,
    yMax = 100,
    yMin = -100,
    initType = Modelica.Blocks.Types.Init.InitialState,
    xi_start = 0) "PI tension controller on the infeed drive"
    annotation(Placement(transformation(origin = {-110, 150}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.Ramp startup(height=world.lineSpeed,duration=startupDuration)
    "Common surface-speed startup"
    annotation(Placement(transformation(origin={-310,-50},extent={{-10,-10},{10,10}})));
  // Drives
  Modelica.Mechanics.Rotational.Sources.Torque infeedTorqueSource
    annotation(Placement(transformation(origin = {-300, 275}, extent = {{170, -135}, {150, -115}})));
  Modelica.Mechanics.Rotational.Sources.Speed outfeedSpeedSource(exact=true)
    annotation(Placement(transformation(origin = {-65, 115}, extent = {{145, -135}, {165, -115}})));
  Modelica.Mechanics.Rotational.Sources.Speed unwindSpeedSource(exact=true)
    annotation(Placement(transformation(origin = {-67.719, 125}, extent = {{-210, -135}, {-190, -115}})));
  Modelica.Mechanics.Rotational.Sources.Speed rewindSpeedSource(exact=true)
    annotation(Placement(transformation(origin = {-10, 85}, extent = {{220, -135}, {240, -115}})));
  Modelica.Blocks.Sources.RealExpression unwindSpeedRef(y = startup.y/max(unwindDrum.radius, Roll2RollDynamics.Utilities.Types.lengthTol))
    annotation(Placement(transformation(origin = {-67.719, 125}, extent = {{-250, -135}, {-230, -115}})));
  Modelica.Blocks.Sources.RealExpression rewindSpeedRef(y = startup.y/max(rewindDrum.radius, Roll2RollDynamics.Utilities.Types.lengthTol))
    annotation(Placement(transformation(origin = {-10, 85}, extent = {{180, -135}, {200, -115}})));
  Modelica.Blocks.Sources.RealExpression outfeedSpeedRef(y = startup.y/(world.rollRadius + world.web.thickness/2))
    annotation(Placement(transformation(origin = {-65, 115}, extent = {{105, -135}, {125, -115}})));

initial equation
  assert(startupDuration > 0, "Startup duration must be positive");
equation
  // Structural supports
  connect(dancerBase.frame_b, dancerPrismatic.frame_a)
    annotation(Line(points = {{0, -9.465}, {0, 9.465}}, color = {95, 95, 95}, thickness = 0.5, origin = {-80, -10.535}));
  connect(dancerPrismatic.frame_b, dancer.frame_support)
    annotation(Line(points = {{0, -18.035}, {0, 18.035}}, color = {95, 95, 95}, thickness = 0.5, origin = {-80, 36.965}));
  connect(dancerPrismatic.axis, dancerSuspension.flange_a)
    annotation(Line(points = {{-12, -4.802}, {-12, 3.267}, {12, 3.267}, {12, -1.733}}, color = {0, 127, 0}, origin = {-62, 21.733}));
  connect(dancerPrismatic.support, dancerSuspension.flange_b)
    annotation(Line(points = {{-13.2, 4.958}, {-4.2, 4.958}, {-4.2, -4.972}, {10.8, -4.972}, {10.8, 0.028}}, color = {0, 127, 0}, origin = {-60.8, -0.028}));
  // Web chain (tangency frames)
  connect(unwindDrum.frame_t, belt1.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-220, 65}));
  connect(belt1.frame_b, infeed.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-180, 65}));
  connect(infeed.frame_b, belt2.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-140, 65}));
  connect(belt2.frame_b, dancer.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-100, 65}));
  connect(dancer.frame_b, belt3.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-60, 65}));
  connect(belt3.frame_b, idler.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-20, 65}));
  connect(idler.frame_b, belt4.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {20, 65}));
  connect(belt4.frame_b, guide.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {60, 65}));
  connect(guide.frame_b, belt5.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {100, 65}));
  connect(belt5.frame_b, outfeed.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {140, 65}));
  connect(outfeed.frame_b, belt6.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {180, 65}));
  connect(belt6.frame_b, rewindDrum.frame_t)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {220, 65}));

  // Tension measurement for the PI controller
  connect(tensionReference.y, controller.u_s)
    annotation(Line(points = {{3.5, 0}, {-3.5, 0}}, color = {0, 0, 127}, origin = {-94.5, 150}));
  connect(belt2.tension, controller.u_m)
    annotation(Line(points = {{-2, -17.325}, {-2, 49.675}}, color = {0, 0, 127}, origin = {-108, 88.325}));

  // Drives
  connect(controller.y, infeedTorqueSource.tau)
    annotation(Line(points = {{3.5, 0}, {-3.5, 0}}, color = {0, 0, 127}, origin = {-124.5, 150}));
  connect(infeedTorqueSource.flange, infeed.flange_a)
    annotation(Line(points = {{6.667, 28.333}, {-3.333, 28.333}, {-3.333, -56.667}}, origin = {-156.667, 121.667}));
  connect(outfeedSpeedRef.y, outfeedSpeedSource.w_ref)
    annotation(Line(points = {{-8.5, 0}, {8.5, 0}}, color = {0, 0, 127}, origin = {68.5, -10}));
  connect(outfeedSpeedSource.flange, outfeed.flange_a)
    annotation(Line(points = {{-40, -25}, {20, -25}, {20, 50}}, origin = {140, 15}));
  connect(unwindSpeedRef.y, unwindSpeedSource.w_ref)
    annotation(Line(points = {{-8.5, 0}, {8.5, 0}}, color = {0, 0, 127}, origin = {-288.219, -0}));
  connect(unwindSpeedSource.flange, unwindDrum.flange_a)
    annotation(Line(points = {{-11.813, -18.333}, {5.906, -18.333}, {5.906, 36.667}}, origin = {-245.906, 18.333}));
  connect(rewindSpeedRef.y, rewindSpeedSource.w_ref)
    annotation(Line(points = {{-8.5, 0}, {8.5, 0}}, color = {0, 0, 127}, origin = {199.5, -40}));
  connect(rewindSpeedSource.flange, rewindDrum.flange_a)
    annotation(Line(points = {{-6.667, -31.667}, {3.333, -31.667}, {3.333, 63.333}}, origin = {236.667, -8.333}));

  annotation(
    experiment(StartTime = 0, StopTime = 20, Interval = 0.002, Tolerance = 1e-7),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {
        Text(textColor = {64, 64, 64}, extent = {{-100, -96}, {100, -66}}, textString = "%name")}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-360, -160}, {270, 245.353}}, initialScale = 0.1, grid = {10, 10})),
    Documentation(figures = {
      Figure(title = "Lateral tracking and normal entry", identifier = "lateralTracking", preferred = true,
        caption = "Results for the yaw setting of this simulation; the documentation's yaw comparison combines separate runs",
        plots = {
          Plot(title = "Web lateral offset at the idler",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Lateral offset", unit = "mm"),
            curves = {Curve(x = time, y = idlerLateralOffset, legend = "Current run")}),
          Plot(title = "Normal entry at the yawed idler",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Roll-relative entry skew sine", unit = "1"),
            curves = {
              Curve(x = time, y = idlerSpanAxisSkew, legend = "Current run"),
              Curve(x = time, y = normalEntry, legend = "Normal entry")})}),
      Figure(title = "Controlled span tension", identifier = "spanTension", preferred = true,
        caption = "Current-run tension and setpoint; the documentation also compares a separate aligned-idler run",
        plots = {Plot(title = "Span tension during startup",
          x = Axis(label = "Time", unit = "s"), y = Axis(label = "Span tension", unit = "N"),
          curves = {
            Curve(x = time, y = belt2.tension, legend = "Current run"),
            Curve(x = time, y = tensionReference.y, legend = "Setpoint")})})}, info = "<html>
<h4>A converting line with a misaligned idler</h4>
<p>This model explores yaw-driven lateral tracking and tension control.
An unwind, infeed, dancer, idler, guide, outfeed and rewind transport
125 &micro;m PET, 600 mm wide, at 2 m/s and 400 N nominal tension.
<a href=\"modelica://Roll2RollDynamics.GettingStarted\">Getting Started</a>
explains the setup. The outfeed and winders share a speed ramp set by
<code>startupDuration</code> (2 s); infeed torque controls <code>belt2.tension</code>.</p>
<p>Figure 1 shows <code>idler.yaw = 0.004</code> rad, with spans and wraps
coloured by machine-direction stress. This illustrative assembly differs
from Good and Beisel's test machine [3] in material and boundary conditions.
The PET modulus gives <code>EA = 270</code> kN [4] and nominal stress
5.33 MPa. The 4 mm/m yaw represents a deliberate fault; [2, 5] give
practical alignment and tension guidance.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/MisalignedIdlerLine/MisalignmentGif.gif\"
     width=\"600\" alt=\"The converting line running with a yawed idler\">
<strong>Figure 1:</strong> The line running with the yawed idler.</p>
<p>Figure 2 compares four yaw settings. At 4 mrad,
<code>idlerLateralOffset</code> settles near 2.23 mm, leaving about 27.8 mm
of the initial 30 mm edge margin on the 660 mm roll. Set <code>idler.yaw = 0</code>
to remove lateral walk. <code>idlerSpanAxisSkew</code>, the incoming span
direction projected along the yawed idler axis, approaches zero for normal
entry [1]; world-y direction alone does not measure this.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/MisalignedIdlerLine/lateral-tracking.png\"
     width=\"600\" alt=\"Lateral offsets for four yaw angles and normal entry at the 4 mrad idler\">
<strong>Figure 2:</strong> Lateral tracking and normal entry.</p>
<p>Figure 3 shows nearly identical aligned and yawed tension responses.
At 20 s, <code>belt2.tension</code> is about 397.45 N, still 2.55 N below
the 400 N setpoint; zero steady-state error is not demonstrated.
The dancer starts preloaded against nominal tension and supported weight.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/MisalignedIdlerLine/tension-hold.png\"
     width=\"600\" alt=\"Measured span tension remaining below the 400 N setpoint at 20 seconds\">
<strong>Figure 3:</strong> Controlled-span tension and its remaining tracking error.</p>
<p>Related examples isolate
<a href=\"modelica://Roll2RollDynamics.Examples.OpenBeltDrive\">OpenBeltDrive</a>
geometry and capstan capacity,
<a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>
normal-entry kinematics [1], and
<a href=\"modelica://Roll2RollDynamics.Examples.SlipAndTractionCapacity\">SlipAndTractionCapacity</a>
regularized friction and nip loading.</p>
<h4>Limitations</h4>
<p>Ideal-speed drives omit torque limits and bandwidth; the ramp has abrupt
acceleration changes. Air entrainment, damage and spatial friction variation
are absent, so no safe ramp rate or wrinkle-free envelope follows.</p>
<p>Spans retain axial elasticity, damping, inertia and separate end tensions,
but no bending, cross-width stress or buckling [2, 3]. Each roll has one
lateral offset shared by entry and exit. Keep the web taut and mount
orientations fixed. For a separate plate analysis, use
<code>idlerSpanLength</code>, <code>idlerSpanTension</code>,
<code>idlerSpanStress</code> and <code>idlerSpanAspect</code>;
<code>idlerSpanSkewSin</code> is a direction component, not shear strain.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/30409\">1</a>]
J. J. Shelton, &quot;Lateral Dynamics of a Moving Web&quot;, PhD thesis,
Oklahoma State University, 1968. The normal entry law, which fixes the skew the
span settles at. Accessed 2026-09-04.</li>
<li>[<a href=\"https://library.rolltoroll.org/content/files/2024/03/roisum_-_roller_align_standards_-_abstract.pdf\">2</a>]
D. R. Roisum, &quot;Roller Alignment - Standards&quot;, Finishing Technologies,
Inc., 2012. Published alignment tolerances, the hundredfold difference between
in-plane and out-of-plane tolerance, and the failure modes that need a plate
model. Accessed 2026-09-04.</li>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/321847\">3</a>]
J. K. Good and J. A. Beisel, &quot;Buckling of Orthotropic Webs in Process
Machinery&quot;, Proc. International Conference on Web Handling, Oklahoma State
University, 2003. Buckling of a tensioned span under misalignment, and the
criteria that decide when a skewed span wrinkles.
<a href=\"https://hdl.handle.net/20.500.14446/321847\">Source</a>.
See PDF pages 8&ndash;11 for the isolated-span analysis and test apparatus.
Accessed 2026-09-07.</li>
<li>[<a href=\"https://pronatindustries.com/wp-content/uploads/2014/07/Mylar_A_50_to_125mic.pdf\">4</a>]
DuPont Teijin Films, &quot;Mylar polyester film, Mylar A 50 - 125 &micro;m,
Product Information&quot;. Modulus and tensile strength of the 125 &micro;m
grade. Accessed 2026-09-04.</li>
<li>[<a href=\"https://www.webhandling.com/tension-0201/\">5</a>]
T. J. Walker, &quot;The Right Tension&quot;, TJWalker + Associates, Inc.
Practical web tension ranges and the tenth-of-break starting rule. Accessed
2026-09-04.</li>
</ul>
</html>"));
end MisalignedIdlerLine;
