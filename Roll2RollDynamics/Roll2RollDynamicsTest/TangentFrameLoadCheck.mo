within Roll2RollDynamicsTest;
package TangentFrameLoadCheck "Tangency support force and moment regressions"
  extends Modelica.Icons.ExamplesPackage;

  model LocalLoad "Prescribed force and moment resolved in the tangency frame"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Modelica.Units.SI.Force force[3]=zeros(3) "Applied local force";
    parameter Modelica.Units.SI.Torque moment[3]=zeros(3) "Applied local moment";
    //Physical connectors
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame "Loaded frame"
      annotation(Placement(visible = true, transformation(origin = {-100, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    frame.f = -force;
    frame.t = -moment;
    annotation(Documentation(info = "<html><h4>Local load</h4><p>Helper boundary that applies prescribed local force and moment to a connected tangency adapter. It supplies the load used by parent tests and is not a standalone scenario.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-140, -100}, {100, 100}})));
  end LocalLoad;

  partial model Fixture "Compare the adapter with independent rigid-load balances"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Modelica.Units.SI.Angle skew=Modelica.Constants.pi/6
      "Span angle across the roll axis";
    parameter Modelica.Units.SI.Angle angle=0 "Tangency angle in the roll plane";
    parameter Integer direction=1 "Directed circumferential travel";
    parameter Boolean transmitAxialMoment=false "Transmit the full offset moment";
    parameter Modelica.Units.SI.Force force[3]={0,10,0} "Applied local force";
    parameter Modelica.Units.SI.Torque moment[3]=zeros(3) "Applied local moment";
    final parameter Modelica.Units.SI.Angle turn=
      angle + (if direction < 0 then Modelica.Constants.pi else 0) "Directed tangent turn";
    final parameter Real tangent[3]={cos(turn)*cos(skew),sin(skew),-sin(turn)*cos(skew)}
      "Local tangent expressed in the centre frame";
    final parameter Real widthDirection[3]={-cos(turn)*sin(skew),cos(skew),sin(turn)*sin(skew)}
      "Local transverse direction expressed in the centre frame";
    final parameter Real normal[3]={sin(turn),0,cos(turn)}
      "Local normal expressed in the centre frame";
    final parameter Modelica.Units.SI.Force expectedForce[3]=
      force[1]*tangent+force[2]*widthDirection+force[3]*normal
      "Applied force independently resolved in the centre frame";
    final parameter Modelica.Units.SI.Torque expectedMoment[3]=
      moment[1]*tangent+moment[2]*widthDirection+moment[3]*normal
      "Applied moment independently resolved in the centre frame";
    final parameter Modelica.Units.SI.Torque fullOffset[3]=
      {-0.15*expectedForce[2],0.15*expectedForce[1],0}
      "Moment of the complete force about the roll centre";
    final parameter Modelica.Units.SI.Torque excludedAxialMoment=
      if transmitAxialMoment then 0 else 0.15*force[1]*tangent[1]
      "Only the longitudinal force moment already carried by shaft coupling";
    //Components
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false)
      "Inertial reference"
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation base(
      n={1,2,3},angle=37,animation=false)
      "Rotate the centre frame so world and local components differ"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.TangentFrame adapter(
      r={0,0,0.15},angle=angle,direction=direction,
      spanDirection=Modelica.Mechanics.MultiBody.Frames.resolve1(base.frame_b.R,tangent),
      transmitAxialMoment=transmitAxialMoment)
      "Tangency adapter under test"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    LocalLoad load(force=force,moment=moment) "Independent spatial load"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, base.frame_a)
      annotation(Line(visible = true, points = {{-150, 80}, {-90, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(base.frame_b, adapter.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(adapter.frame_b, load.frame)
      annotation(Line(visible = true, points = {{10, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(adapter.skew-skew)<1e-12,
      "Tangency skew must be computed from the supplied world direction");
    assert(Modelica.Math.Vectors.length(adapter.frame_a.f+expectedForce)<1e-9,
      "Tangency force transformation violates spatial force balance");
    assert(Modelica.Math.Vectors.length(adapter.frame_a.t+expectedMoment
      +fullOffset-{0,excludedAxialMoment,0})<1e-9,
      "Tangency adapter lost a transverse-load offset moment or a boundary moment");
    annotation(experiment(StopTime=0.01,Tolerance=1e-9),
      Documentation(info = "<html><h4>Tangency load fixture</h4><p>Partial helper fixture that uses independent trigonometric basis vectors to resolve forces and moments through a rotated tangency adapter. The recovered skew and housing reactions must match the independent spatial balances, retaining all boundary and transverse-force moments; only the longitudinal force moment about the shaft may be excluded when <code>transmitAxialMoment=false</code>. Concrete derived tests select the loads and winding direction.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
  end Fixture;

  model SkewTransverse "A transverse load on a skewed span produces shaft-axis support moment"
    extends Fixture;
  equation
    assert(abs(adapter.frame_a.t[2]-0.75)<1e-9,
      "The 10 N transverse force requires a 0.75 N m housing reaction");
    annotation(Documentation(info = "<html><h4>Skew transverse load</h4><p>Applies a 10 N transverse force at 30 degrees of skew and a 0.15 m offset, giving a -5 N circumferential component. The housing must carry its 0.75 N m shaft-axis reaction within 1e-9 N m and satisfy the inherited spatial load balances.</p></html>"));
  end SkewTransverse;

  model LongitudinalOnly "Longitudinal shaft torque is excluded while other moments remain"
    extends Fixture(force={10,0,0});
    annotation(Documentation(info = "<html><h4>Longitudinal load only</h4><p>Applies a pure longitudinal force with default exclusion of its shaft-axis moment, which is carried by transport coupling. The support must retain all remaining offset-moment components and satisfy the inherited force, moment and skew assertions.</p></html>"));
  end LongitudinalOnly;

  model MixedForceAndMoment "Nonzero force and boundary moment survive frame transformations"
    extends Fixture(force={7,11,13},moment={2,3,4},angle=0.4);
    annotation(Documentation(info = "<html><h4>Mixed force and moment</h4><p>Applies nonzero components in every local force and moment direction with both connected frames rotated. The transformed loads and retained offset moments must match the independent fixture balances.</p></html>"));
  end MixedForceAndMoment;

  model FullMoment "Full moment transmission retains the complete rigid offset load"
    extends Fixture(force={7,11,13},moment={2,3,4},angle=-0.3,transmitAxialMoment=true);
    annotation(Documentation(info = "<html><h4>Full moment</h4><p>Enables <code>transmitAxialMoment</code> under mixed force and moment loading. The adapter must transmit the complete rigid offset moment and satisfy the independent spatial force and moment balances.</p></html>"));
  end FullMoment;

  model ReverseDirection "Reverse winding preserves the same physical load balance"
    extends Fixture(force={7,11,13},moment={2,3,4},angle=0.7,direction=-1);
    annotation(Documentation(info = "<html><h4>Reverse direction</h4><p>Applies mixed force and moment loading with reverse winding. The tangent frame must include the reverse-travel turn while retaining transverse support moments and satisfying the independent spatial balances.</p></html>"));
  end ReverseDirection;

  model MovingSkew "A changing supplied direction preserves skew and angular velocity"
    extends Modelica.Icons.Example;
    //Parameters
    final parameter Modelica.Units.SI.Angle turn=0.4+Modelica.Constants.pi
      "Reverse-travel tangent turn";
    //Variables with Binding Equations
    Modelica.Units.SI.Angle expectedSkew=0.2*sin(time) "Prescribed cross-machine angle";
    Real tangent[3]={cos(turn)*cos(expectedSkew),sin(expectedSkew),
      -sin(turn)*cos(expectedSkew)} "Independent material direction in the roll frame";
    //Components
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false)
      "Inertial reference"
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation base(
      n={1,2,3},angle=37,animation=false) "Rotated roll-centre frame"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.TangentFrame adapter(
      r={0,0,0.15},angle=0.4,direction=-1,
      spanDirection=Modelica.Mechanics.MultiBody.Frames.resolve1(base.frame_b.R,tangent))
      "Adapter with time-varying skew"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    LocalLoad load(force={7,11,13},moment={2,3,4}) "Applied spatial load"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, base.frame_a)
      annotation(Line(visible = true, points = {{-150, 80}, {-90, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(base.frame_b, adapter.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(adapter.frame_b, load.frame)
      annotation(Line(visible = true, points = {{10, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(adapter.skew-expectedSkew)<1e-12,
      "Explicit skew must follow the supplied direction through zero");
    assert(Modelica.Math.Vectors.length(adapter.frame_b.R.w-{0,0,0.2*cos(time)})<1e-12,
      "Tangency angular velocity must retain the derivative of the supplied skew");
    annotation(experiment(StopTime=4,Tolerance=1e-9),Documentation(info = "<html><h4>Moving skew</h4><p>Supplies a sinusoidal span direction to a rotated reverse-travel tangency frame. Recovered skew and frame angular velocity must follow the imposed angle and its derivative through zero, within 1e-12.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
  end MovingSkew;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Checks the tangency adapter using independently
    resolved forces and moments, without a Belt or parent-level correction.
    </p></html>"));
end TangentFrameLoadCheck;
